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
NMEA_RE = re.compile(r"NMEA (\$([A-Z]{2})([A-Z]{3})\S*)")

# 見やすさのため、図の色は 2 つの条件で固定する。
SLOW_C, FAST_C = "#c1442e", "#2e7d5b"


def load(path: Path):
    """(PPS エッジ時刻, [(センテンス時刻, 種別, バイト長)]) を返す。"""
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
            # 中身は座標を含むので保持しない。占有率に要るのは長さだけである (+2 は CRLF)。
            sentences.append((t, n.group(3), len(n.group(1)) + 2))
    return edges, sentences


def margins(edges, sentences, kind):
    """種別 `kind` のセンテンスについて (直前エッジからの経過, 次エッジまでの残り)。"""
    out = []
    for t, k, _n in sentences:
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
    for t, k, _n in sentences:
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


def occupancy(edges, sentences, baud):
    """NMEA が UART を毎秒どれだけ占有しているか。(占有率, 1 秒あたりの文字数) を返す。

    長さは実際に届いたセンテンスそのものから取る。種別名から概算していたこともあったが、
    GSV のように衛星数で伸びる文があるので、それでは占有率が当たらない。1 文字 10 bit
    (8N1) で数える。
    """
    span = edges[-1] - edges[0]
    n = sum(b for t, _k, b in sentences if edges[0] <= t <= edges[-1])
    return n * 10 / span / baud, n / span


def fig_timeline(slow, fast, notes, path: Path):
    """代表的な 1 秒に、実測のセンテンス到着をそのまま並べる。

    注釈は全部 axes の内側に置く。外に出すと x 軸ラベルや目盛と重なるし、右端の
    センテンスでは図からはみ出す。位置に応じて揃えを変えて、線やラベル同士の衝突も避ける。
    """
    fig, axes = plt.subplots(2, 1, figsize=(10, 4.6), sharex=True)
    for ax, (label, items, colour, note) in zip(
        axes,
        [
            ("9600 baud", slow, SLOW_C, notes[0]),
            ("115200 baud", fast, FAST_C, notes[1]),
        ],
    ):
        for x, text, ha, dx in (
            (0.0, "PPS edge", "left", 0.012),
            (1.0, "next PPS edge", "right", -0.012),
        ):
            ax.axvline(x, color="#333", lw=2)
            # 線の上に文字を乗せない。線から少しずらして、外向きでなく内向きに逃がす。
            ax.text(x + dx, 1.42, text, ha=ha, va="top", fontsize=9, color="#333")

        for off, kind in items:
            hot = kind in ("RMC", "ZDA")
            ax.plot(
                [off, off],
                [0.18, 0.62] if hot else [0.26, 0.54],
                color=colour if hot else "#aaa",
                lw=2.4 if hot else 1.2,
            )

        # 時刻センテンスの注釈は上へ、二本を縦にずらして重ならないようにする。
        for kind, y in (("RMC", 1.06), ("ZDA", 0.80)):
            hit = [o for o, k in items if k == kind]
            if not hit:
                continue
            off = hit[0]
            # 右端に近ければ右揃えにして、図の外へ出さない。
            ha = "right" if off > 0.62 else "left"
            dx = -0.015 if ha == "right" else 0.015
            ax.annotate(
                f"{kind}: {(1.0 - off) * 1000:.0f} ms to the next edge",
                xy=(off, 0.62),
                xytext=(off + dx, y),
                ha=ha,
                va="center",
                fontsize=8.5,
                color=colour,
                arrowprops=dict(arrowstyle="-", color=colour, lw=0.8, alpha=0.6),
            )

        ax.text(0.5, 0.06, note, ha="center", va="bottom", fontsize=9, color=colour)
        ax.set_ylim(0, 1.5)
        ax.set_yticks([])
        ax.set_ylabel(label, rotation=0, ha="right", va="center", fontsize=10)
        for side in ("top", "right", "left"):
            ax.spines[side].set_visible(False)

    axes[-1].set_xlim(-0.03, 1.06)
    axes[-1].set_xlabel("seconds after a PPS edge")
    fig.suptitle(
        "One second of NMEA, as logged: every sentence arrival between two PPS edges",
        fontsize=12,
    )
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"wrote {path}")


def fig_margin(slow_m, fast_m, path: Path):
    """次エッジまでの残りの分布。9600 は二峰、115200 は一峰。

    どちらの系列も RMC である。本文では ZDA も扱うので、凡例でセンテンス種別まで明示して
    「センテンスによらない性質」と誤読されないようにする。
    """
    fig, ax = plt.subplots(figsize=(9, 3.6))
    bins = [i / 50 for i in range(51)]
    ax.hist(
        [m * 1000 for _, m in slow_m],
        bins=[b * 1000 for b in bins],
        color=SLOW_C,
        alpha=0.75,
        label=f"RMC @ 9600 baud (n={len(slow_m)})",
    )
    ax.hist(
        [m * 1000 for _, m in fast_m],
        bins=[b * 1000 for b in bins],
        color=FAST_C,
        alpha=0.75,
        label=f"RMC @ 115200 baud (n={len(fast_m)})",
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

    slow_u, slow_c = occupancy(slow_e, slow_s, 9600)
    fast_u, fast_c = occupancy(fast_e, fast_s, 115200)
    print(f"9600   {slow_c:.0f} chars/s -> {slow_u * 100:.0f}% of the link")
    print(f"115200 {fast_c:.0f} chars/s -> {fast_u * 100:.0f}% of the link")

    for name, e, s in (("9600  ", slow_e, slow_s), ("115200", fast_e, fast_s)):
        it = one_second(e, s)
        gaps = [b[0] - a[0] for a, b in zip(it, it[1:])]
        print(
            f"{name} burst {it[0][0] * 1000:.0f}..{it[-1][0] * 1000:.0f} ms "
            f"(span {(it[-1][0] - it[0][0]) * 1000:.0f} ms, n={len(it)}, "
            f"max gap {max(gaps) * 1000:.0f} ms)"
        )
        for kind in ("RMC", "ZDA"):
            hit = [o for o, k in it if k == kind]
            if hit:
                print(f"{name}   {kind} at {hit[0] * 1000:.0f} ms, {(1 - hit[0]) * 1000:.0f} ms to next edge")

    fig_timeline(
        one_second(slow_e, slow_s),
        one_second(fast_e, fast_s),
        [f"{slow_u * 100:.0f}% of the link is NMEA", f"{fast_u * 100:.0f}% of the link is NMEA"],
        OUT / "fig-burst-timing.png",
    )
    fig_margin(slow_rmc, fast_rmc, OUT / "fig-margin.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
