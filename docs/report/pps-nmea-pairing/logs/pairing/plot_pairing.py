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

# 図には実機が出している電文名で書く。ログの突き合わせは 3 文字の種別で行うが、
# 表示まで 3 文字にすると GP 始まりだと読まれる (この個体の RMC は GNRMC である)。
FULL = {"RMC": "GNRMC", "ZDA": "GPZDA", "GGA": "GPGGA", "GST": "GPGST"}


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


def occupancy(edges, sentences, baud):
    """NMEA が UART を毎秒どれだけ占有しているか。(占有率, 1 秒あたりの文字数) を返す。

    長さは実際に届いたセンテンスそのものから取る。種別名から概算していたこともあったが、
    GSV のように衛星数で伸びる文があるので、それでは占有率が当たらない。1 文字 10 bit
    (8N1) で数える。
    """
    span = edges[-1] - edges[0]
    n = sum(b for t, _k, b in sentences if edges[0] <= t <= edges[-1])
    return n * 10 / span / baud, n / span


def one_cycle(edges, sentences):
    """代表的な 1 サイクルの到着を、その直前の PPS エッジからの経過 (ms) で返す。

    切れ目は GGA に取る。サイクルの先頭が GGA であることは、同じ UTC 秒を載せた
    センテンスを突き合わせて確かめてある (measure_pairing.py)。

    エッジからエッジの「窓」で切ってはいけない。9600 bps ではサイクルが 1 秒より長いので、
    窓には前後のサイクルのセンテンスが混ざる。混ざったまま並べると、前サイクルの ZDA が
    窓の冒頭に写り、ZDA が RMC より先に届いているように見える。実際の送出順は逆である。

    返す経過は 1000 ms を超えうる。超えている部分が、そのサイクルが次のエッジを跨いだ量に
    あたる。
    """
    chunks, cur = [], None
    for t, kind, _n in sentences:
        if kind == "GGA":
            if cur:
                chunks.append(cur)
            cur = []
        if cur is not None:
            cur.append((t, kind))
    if cur:
        chunks.append(cur)
    if not chunks:
        return []
    # たまたま短い/長いサイクルを代表にしないよう、本数が中央値のものを選ぶ。
    target = sorted(len(c) for c in chunks)[len(chunks) // 2]
    for c in chunks:
        if len(c) != target:
            continue
        prev = [e for e in edges if e <= c[0][0]]
        if not prev:
            continue
        return [((t - prev[-1]) * 1000, k) for t, k in c]
    return []


def fig_timeline(panels, title, path: Path):
    """代表的な 1 サイクルの到着を、直前の PPS エッジからの経過で並べる。

    条件ごとに 1 枚ずつ描く。1 枚に 9600 と 115200 を並べると、9600 の話をしている段で
    115200 の結果まで目に入ってしまい、図が本文より先に答えを出す。

    注釈は全部 axes の内側に置く。外に出すと x 軸ラベルや目盛と重なるし、右端の
    センテンスでは図からはみ出す。位置に応じて揃えを変えて、線やラベル同士の衝突も避ける。
    """
    fig, axes = plt.subplots(
        len(panels), 1, figsize=(10, 2.9 * len(panels)), sharex=True, squeeze=False
    )
    for ax, (label, items, colour) in zip(axes[:, 0], panels):
        for x, text, ha, dx in (
            (0.0, "PPS edge", "left", 10),
            (1000.0, "next PPS edge", "right", -10),
        ):
            ax.axvline(x, color="#333", lw=2)
            ax.text(x + dx, 1.42, text, ha=ha, va="top", fontsize=9, color="#333")

        for off, kind in items:
            hot = kind in ("RMC", "ZDA")
            ax.plot(
                [off, off],
                [0.18, 0.62] if hot else [0.26, 0.54],
                color=colour if hot else "#aaa",
                lw=2.4 if hot else 1.2,
            )

        for kind, y in (("RMC", 1.06), ("ZDA", 0.80)):
            hit = [o for o, k in items if k == kind]
            if not hit:
                continue
            off = hit[0]
            ha = "right" if off > 700 else "left"
            dx = -14 if ha == "right" else 14
            ax.annotate(
                f"{FULL.get(kind, kind)} at {off:.0f} ms",
                xy=(off, 0.62),
                xytext=(off + dx, y),
                ha=ha,
                va="center",
                fontsize=8.5,
                color=colour,
                arrowprops=dict(arrowstyle="-", color=colour, lw=0.8, alpha=0.6),
            )

        ax.set_ylim(0, 1.5)
        ax.set_yticks([])
        ax.set_ylabel(label, rotation=0, ha="right", va="center", fontsize=10)
        for side in ("top", "right", "left"):
            ax.spines[side].set_visible(False)

    axes[-1, 0].set_xlim(-40, 1260)
    axes[-1, 0].set_xlabel("ms since the PPS edge before this cycle started")
    fig.suptitle(title, fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"wrote {path}")


# 境界のすぐ近くとみなす幅。ここに落ちたセンテンスは、バーストのわずかな揺れで
# 境界の反対側へ移りうる。
NEAR_MS = 100


def fig_margin(series, path: Path):
    """時刻センテンスが届いてから次の PPS エッジまでの残り時間の分布。

    条件ごとに段を分ける。1 枚に重ねると、山がどちらの系列のものか読み取れないうえ、
    「センテンスの種別によらない性質」と誤読されやすい。

    0 ms と 1000 ms は同じ境界を両側から見た値である。残りが 0 に近いセンテンスは
    エッジを跨ぐ寸前におり、1000 に近いセンテンスは跨いだ直後にいる。どちらも
    「境界の際」なので、両端に同じ幅の帯を敷いて、そこに入った割合を出す。
    """
    fig, axes = plt.subplots(
        len(series), 1, figsize=(9, 1.9 * len(series)), sharex=True, squeeze=False
    )
    for ax, (label, margins, colour) in zip(axes[:, 0], series):
        v = [m * 1000 for _, m in margins]
        # 両端は同じ境界だが、跨ぐ前か後かで対応付けの外れ方が変わる。合計だけ出すと
        # 「半々で転ぶ」(RMC) と「いつも同じ向きに外す」(ZDA) が同じ数字に潰れるので、
        # 帯を色で分けて別々に数える。
        before = sum(1 for x in v if x < NEAR_MS)
        after = sum(1 for x in v if x > 1000 - NEAR_MS)
        ax.axvspan(0, NEAR_MS, color="#b5762f", alpha=0.16, lw=0)
        ax.axvspan(1000 - NEAR_MS, 1000, color="#2f6fb5", alpha=0.16, lw=0)
        ax.hist(v, bins=[i * 20 for i in range(51)], color=colour)
        ax.set_ylabel(label, rotation=0, ha="right", va="center", fontsize=10)
        ax.text(
            130,
            ax.get_ylim()[1] * 0.84,
            f"about to cross: {before / len(v) * 100:.0f}%"
            f"    just crossed: {after / len(v) * 100:.0f}%"
            f"    (n={len(v)}, min {min(v):.0f} ms)",
            ha="left",
            fontsize=9.5,
            color=colour,
        )
        for side in ("top", "right", "left"):
            ax.spines[side].set_visible(False)
        ax.set_yticks([])

    # 説明は最終段の空いている左側に置く。上段に置くと、その段の数値と重なる。
    axes[-1, 0].text(
        130,
        axes[-1, 0].get_ylim()[1] * 0.52,
        "shaded = within 100 ms of an edge.  amber is about to cross it,\n"
        "blue has already crossed and is measured to the following edge.",
        ha="left",
        va="top",
        fontsize=9,
        color="#333",
    )
    axes[-1, 0].set_xlim(0, 1000)
    axes[-1, 0].set_xlabel(
        "margin from the time sentence to the next PPS edge (ms)\n"
        "0 = about to cross an edge,  1000 = just crossed one"
    )
    fig.suptitle(
        "How close to a PPS edge does the time sentence arrive?", fontsize=12
    )
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
        it = one_cycle(e, s)
        gaps = [b[0] - a[0] for a, b in zip(it, it[1:])]
        print(
            f"{name} cycle {it[0][0]:.0f}..{it[-1][0]:.0f} ms after the preceding edge "
            f"(span {it[-1][0] - it[0][0]:.0f} ms, n={len(it)}, "
            f"max gap {max(gaps):.0f} ms)"
        )
        for kind in ("RMC", "ZDA"):
            hit = [o for o, k in it if k == kind]
            if hit:
                print(f"{name}   {kind} at {hit[0]:.0f} ms ({hit[0] - 1000:+.0f} ms relative to the next edge)")

    fig_timeline(
        [
            ("9600 baud", one_cycle(slow_e, slow_s), SLOW_C)
        ],
        "One cycle of NMEA at 9600 baud, as logged",
        OUT / "fig-burst-9600.png",
    )
    fig_timeline(
        [
            ("115200 baud", one_cycle(fast_e, fast_s), FAST_C)
        ],
        "The same cycle at 115200 baud",
        OUT / "fig-burst-115200.png",
    )
    fig_margin(
        [
            ("GNRMC\n@ 9600", slow_rmc, SLOW_C),
            ("GPZDA\n@ 9600", slow_zda, SLOW_C),
        ],
        OUT / "fig-margin.png",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
