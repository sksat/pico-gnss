# /// script
# requires-python = ">=3.11"
# dependencies = ["matplotlib"]
# ///
"""レポート `docs/report/ntp-stratum1/` の図と数値を作る。

    uv run docs/report/ntp-stratum1/logs/paths/plot_paths.py

生データは repo top の `logs/20260819-ntp-wired/` から読む (コミットしない)。

    paths.log      受信側。scripts/ntp_broadcast_listen.py の 1 行ずつの出力
    rtt-10123.log  送信側。probe 経由の firmware ログ

出力は `docs/report/ntp-stratum1/` 直下の PNG。図中のラベルは英語にしてある。日本語フォントが
入っていない環境で豆腐になるのを避けるため。

本文の数値はここが出したものだけを使う。図の注釈も同じ計算から取るので、片方だけ古くならない。
"""

import cmath
import re
import statistics
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[5]
RAW = REPO / "logs" / "20260819-ntp-wired"
OUT = Path(__file__).resolve().parents[2]

WIRED, WIRELESS = "eth0", "wlp1s0"
WIRED_C, WIRELESS_C = "#2e7d5b", "#c1442e"
# IEEE 802.11 の time unit。ビーコン間隔はこの単位で数える。
TU_MS = 1.024
BEACON_TU = 100
# 10BASE-T は 10 Mbit/s。preamble + SFD はフレームの前に付く 8 byte。
BIT_US = 0.1
PREAMBLE_SFD_BYTES = 8
# 有線側の分布を二つに分ける位置。fig-wired.png の谷を見て選んだ入力であって、結果ではない。
CUT_US = -3900.0

RECV_LINE = re.compile(
    r"^(?P<iface>\S+)\s+\S+\s+mode=(?P<mode>\S+)\s+stratum=(?P<stratum>\d+)\s+"
    r"li=(?P<li>\d+)\s+prec=(?P<prec>\S+)\s+refid=(?P<refid>\S+)\s+"
    r"rootdisp=(?P<rootdisp>\S+)\s+xmt=(?P<xmt>[\d:.]+)\s+recv=(?P<recv>[\d:.]+)\s"
)
TX_LINE = re.compile(
    r"NTPTX n=(?P<n>\d+) target_unix_ns=(?P<target>\d+) sched_ns=(?P<sched>-?\d+) "
    r"tx_lead_ns=\S+ dma_us=(?P<dma>\d+) bytes=(?P<bytes>\d+)"
)


def to_ns(hms: str) -> int:
    hh, mm, rest = hms.split(":")
    ss, frac = rest.split(".")
    return ((int(hh) * 60 + int(mm)) * 60 + int(ss)) * 1_000_000_000 + int(
        frac.ljust(9, "0")[:9]
    )


def load_received():
    recv: dict[str, dict[int, int]] = {}
    header: dict[str, set] = {}
    rows = 0
    for line in (RAW / "paths.log").read_text(errors="replace").splitlines():
        m = RECV_LINE.match(line)
        if not m:
            continue
        rows += 1
        recv.setdefault(m["iface"], {}).setdefault(to_ns(m["xmt"]), to_ns(m["recv"]))
        for field in ("mode", "stratum", "li", "prec", "refid", "rootdisp"):
            header.setdefault(field, set()).add(m[field])
    return recv, header, rows


def load_transmitted():
    """送信側の記録。受信ログとは独立に、何秒ぶん送ったかと送出の遅れを持っている。"""
    log = RAW / "rtt-10123.log"
    if not log.exists():
        return None
    rows = [m.groupdict() for m in TX_LINE.finditer(log.read_text(errors="replace"))]
    return rows or None


def fit_release(idx, wait):
    """放出周期と位相を、全区間の位相のまとまりからフィットする。

    待ちは w(t) = (φ − 1000·t) mod T の形をしている。T が正しければ a_i = (w_i + 1000·t_i) mod T
    が一点に集まるので、単位ベクトルの合成長 R を最大にする T を採る。

    階差の中央値から T を出す手もあるが、1 サンプルあたり 0.05 ms の誤差が 600 秒で 100 ms 積もり、
    位相の再構成が立たなくなる。全区間を一度に使えばその誤差が入らない。
    """

    def concentration(period: float):
        vec = sum(
            cmath.exp(2j * cmath.pi * ((w + 1000.0 * i) % period) / period)
            for w, i in zip(wait, idx)
        )
        return abs(vec) / len(wait), (cmath.phase(vec) / (2 * cmath.pi) * period) % period

    grid = [306.0 + k * 0.0005 for k in range(4001)]
    r, period = max((concentration(p)[0], p) for p in grid)
    _, phase = concentration(period)
    return period, phase, r


def runs_of_losses(seen, got):
    lengths, run = [], 0
    for x in seen:
        if x in got:
            if run:
                lengths.append(run)
            run = 0
        else:
            run += 1
    if run:
        lengths.append(run)
    return lengths


def main() -> int:
    if not (RAW / "paths.log").exists():
        print(f"missing raw log: {RAW / 'paths.log'}", file=sys.stderr)
        return 1

    recv, header, rows = load_received()
    if WIRED not in recv or WIRELESS not in recv:
        print(f"need both interfaces, saw {list(recv)}", file=sys.stderr)
        return 1

    seen = sorted(set(recv[WIRED]) | set(recv[WIRELESS]))
    t0 = seen[0]
    slots = (seen[-1] - t0) // 1_000_000_000 + 1
    holes = [
        (seen[i - 1], seen[i])
        for i in range(1, len(seen))
        if seen[i] - seen[i - 1] != 1_000_000_000
    ]
    print(f"受信ログ {rows} 行、どちらかに届いた秒 {len(seen)} 個、区間は {slots} スロット")
    for a, b in holes:
        print(
            f"  どちらにも来なかった秒: {(b - a) // 1_000_000_000 - 1} 個 "
            f"({(a // 1_000_000_000 + 1) % 86400} から)"
        )
    print("ヘッダは全通で同じ:", {k: sorted(v)[0] for k, v in header.items() if len(v) == 1})
    if any(len(v) > 1 for v in header.values()):
        print("  そろっていない:", {k: sorted(v) for k, v in header.items() if len(v) > 1})

    # --- 送信側 -------------------------------------------------------------------------------
    tx = load_transmitted()
    if tx:
        ns = [int(r["n"]) for r in tx]
        secs = sorted((int(r["target"]) // 1_000_000_000) % 86400 for r in tx)
        lo, hi = t0 // 1_000_000_000, seen[-1] // 1_000_000_000
        window = [s for s in secs if lo <= s <= hi]
        n_gaps = sum(1 for i in range(1, len(ns)) if ns[i] != ns[i - 1] + 1)
        sched = [int(r["sched"]) / 1000.0 for r in tx]
        dma = [int(r["dma"]) for r in tx]
        sizes = {int(r["bytes"]) for r in tx}
        on_wire = max(sizes) + PREAMBLE_SFD_BYTES
        print(
            f"送信側: NTPTX {len(tx)} 行、通し番号の飛び {n_gaps}、"
            f"受信ログと重なるのは {len(window)} 秒 (受信窓の {100 * len(window) / len(seen):.0f}%)"
        )
        print(
            f"  その {len(window)} 秒のうち {WIRED} が受けた "
            f"{sum(1 for s in window if s * 1_000_000_000 in recv[WIRED])}"
        )
        print(
            f"  送出の遅れ (申告した時刻から handover まで): median {statistics.median(sched):.1f} us、"
            f"{min(sched):.1f}–{max(sched):.1f}、sd {statistics.pstdev(sched):.1f}"
        )
        print(
            f"  frame {sorted(sizes)} byte + preamble/SFD {PREAMBLE_SFD_BYTES} byte = {on_wire} byte が"
            f"線に乗るのに {on_wire * 8 * BIT_US:.1f} us"
        )
        print(
            f"  dma_us (送信呼び出しが返るまで): median {statistics.median(dma)}、{min(dma)}–{max(dma)}"
        )

    # --- 経路ごと -----------------------------------------------------------------------------
    offsets = {i: [(x - recv[i][x]) / 1e6 for x in sorted(recv[i])] for i in (WIRED, WIRELESS)}
    print()
    for i in (WIRED, WIRELESS):
        lost = len(seen) - len(recv[i])
        o = offsets[i]
        print(
            f"  {i:<8} 受信 {len(recv[i]):4d} / {len(seen)}  欠落 {lost:3d} ({100 * lost / len(seen):.1f}%)"
            f"   mean {statistics.mean(o) * 1000:+9.1f} us  sd {statistics.pstdev(o) * 1000:8.1f} us"
        )
    print(f"  {WIRELESS} の受信は {WIRED} の部分集合か: {set(recv[WIRELESS]) <= set(recv[WIRED])}")
    mixed = [v for i in (WIRED, WIRELESS) for v in offsets[i]]
    print(
        f"  分けずに混ぜると mean {statistics.mean(mixed) * 1000:+.1f} us  "
        f"sd {statistics.pstdev(mixed) * 1000:.1f} us"
    )

    ow = sorted(v * 1000 for v in offsets[WIRED])
    upper = [v for v in ow if v >= CUT_US]
    lower = [v for v in ow if v < CUT_US]
    print(
        f"  {WIRED} を {CUT_US:.0f} us で切ると 上 {len(upper)} 通 mean {statistics.mean(upper):+.0f} us / "
        f"下 {len(lower)} 通 ({100 * len(lower) / len(ow):.0f}%) mean {statistics.mean(lower):+.0f} us、"
        f"隔たり {statistics.mean(upper) - statistics.mean(lower):.0f} us"
    )

    # 二山が送信側のものかを見る。送出の遅れと、その秒の offset が上下どちらの群かを突き合わせる。
    if tx:
        by_sec = {(int(r["target"]) // 1_000_000_000) % 86400: int(r["sched"]) / 1000.0 for r in tx}
        pairs = [
            (by_sec[s], (s * 1_000_000_000 - recv[WIRED][s * 1_000_000_000]) / 1e3)
            for s in by_sec
            if s * 1_000_000_000 in recv[WIRED]
        ]
        if len(pairs) > 2:
            xs = [a for a, _ in pairs]
            ys = [b for _, b in pairs]
            mx, my = statistics.mean(xs), statistics.mean(ys)
            cov = sum((a - mx) * (b - my) for a, b in pairs) / len(pairs)
            sx, sy = statistics.pstdev(xs), statistics.pstdev(ys)
            lo_s = [a for a, b in pairs if b < CUT_US]
            hi_s = [a for a, b in pairs if b >= CUT_US]
            print(
                f"  送出の遅れ と offset の相関 r={cov / (sx * sy):+.3f} (n={len(pairs)})、"
                f"下群の遅れ median {statistics.median(lo_s) if lo_s else float('nan'):.1f} us / "
                f"上群 {statistics.median(hi_s) if hi_s else float('nan'):.1f} us"
            )

    # 補正を入れたあとの記録があれば、client が読む値がどう動いたかも出す。
    later_line = re.compile(r"^(?P<if>\S+)\s+\S+\s+mode=\S+.*?offset=(?P<off>[-+\d.]+)us")
    for name in ("paths-corrected.log", "paths-sync.log"):
        later = RAW / name
        if not later.exists():
            continue
        vals = []
        for line in later.read_text(errors="replace").splitlines():
            m = later_line.match(line)
            if m and m["if"] == WIRED:
                vals.append(float(m["off"]))
        if vals:
            print(
                f"  {name}: n={len(vals)} median {statistics.median(vals):+.1f} us "
                f"sd {statistics.pstdev(vals):.1f}"
            )

    lengths = runs_of_losses(seen, recv[WIRELESS])
    tally = {n: lengths.count(n) for n in sorted(set(lengths))}
    print(
        f"  {WIRELESS} の欠落 {sum(lengths)} 通 / {len(lengths)} 連: "
        + "、".join(f"{n} 連が {c} 回" for n, c in tally.items())
    )

    # --- 経路の差と放出周期 -----------------------------------------------------------------
    both = [x for x in seen if x in recv[WIRELESS]]
    idx = [(x - t0) // 1_000_000_000 for x in both]
    delta = [(recv[WIRELESS][x] - recv[WIRED][x]) / 1e6 for x in both]
    period, phase, r = fit_release(idx, delta)
    quant = statistics.quantiles(delta, n=100)

    print()
    print(
        f"無線 − 有線 (n={len(delta)}): median {statistics.median(delta):.1f} ms  "
        f"mean {statistics.mean(delta):.1f}  sd {statistics.pstdev(delta):.1f}  "
        f"min {min(delta):.1f}  max {max(delta):.1f}  p99 {quant[98]:.1f}"
    )
    print(
        f"  フィットした放出周期 {period:.4f} ms (R={r:.3f})、含意する勾配 {1000 - 3 * period:.3f} ms/s"
    )
    print(
        f"  ビーコン {BEACON_TU} TU = {BEACON_TU * TU_MS:.1f} ms の {period / (BEACON_TU * TU_MS):.4f} 個ぶん、"
        f"3 個ちょうどとの差 {abs(period - 3 * BEACON_TU * TU_MS) * 1000:.1f} us"
    )
    print(f"  周期を越えて届いたのは {sum(1 for d in delta if d > period)} 通、最大 {max(delta):.0f} ms")

    def predicted(i: int) -> float:
        return (phase - 1000.0 * i) % period

    residual = [((d - predicted(i) + period / 2) % period) - period / 2 for d, i in zip(delta, idx)]
    for tol in (5, 10):
        share = 100 * sum(1 for v in residual if abs(v) < tol) / len(residual)
        print(f"  再構成の |残差| < {tol} ms が {share:.0f}%")

    bins = 8
    width = period / bins
    got = [0] * bins
    lost_n = [0] * bins
    for x in seen:
        k = min(int(predicted((x - t0) // 1_000_000_000) / width), bins - 1)
        (got if x in recv[WIRELESS] else lost_n)[k] += 1
    rates = [100 * lost_n[k] / (got[k] + lost_n[k]) for k in range(bins)]
    print("  待ちごとの欠落 (落ちた通の待ちも再構成して数える):")
    for k in range(bins):
        print(
            f"    {k * width:5.0f}–{(k + 1) * width:5.0f} ms  n={got[k] + lost_n[k]:4d}  欠落 {rates[k]:5.1f}%"
        )

    # --- 図 ------------------------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    ax.hist(ow, bins=50, color=WIRED_C, alpha=0.85)
    ax.set_xlabel("server transmit - host receive (us)")
    ax.set_ylabel("count")
    ax.set_title(f"{WIRED}: mean {statistics.mean(ow):+.0f} us, sd {statistics.pstdev(ow):.0f} us")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT / "fig-wired.png", dpi=130)

    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    ax.hist(delta, bins=40, color=WIRELESS_C, alpha=0.85)
    ax.set_xlabel("wireless - wired (ms)")
    ax.set_ylabel("count")
    ax.set_title(
        f"same frame, two paths (n={len(delta)}): "
        f"median {statistics.median(delta):.0f} ms, sd {statistics.pstdev(delta):.0f} ms"
    )
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT / "fig-difference.png", dpi=130)

    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot([(x - t0) / 1e9 for x in both], delta, ".", ms=4, color=WIRELESS_C)
    ax.set_xlabel("elapsed (s)")
    ax.set_ylabel("wireless - wired (ms)")
    ax.set_title(f"the wait falls {-(1000 - 3 * period):.1f} ms a second, then starts over")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT / "fig-sawtooth.png", dpi=130)

    fig, ax = plt.subplots(figsize=(6.8, 3.6))
    ax.bar(
        [(k + 0.5) * width for k in range(bins)],
        rates,
        width=width * 0.85,
        color=WIRELESS_C,
        alpha=0.85,
    )
    ax.set_xlabel("wait until the AP released it (ms)")
    ax.set_ylabel("did not arrive (%)")
    ax.set_title(f"loss against how long the frame waited (n={len(seen)})")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(OUT / "fig-wait-loss.png", dpi=130)

    win = [x for x in seen if x - t0 < 120 * 1_000_000_000]
    fig, ax = plt.subplots(figsize=(11, 2.2))
    for row, (iface, color) in enumerate(((WIRED, WIRED_C), (WIRELESS, WIRELESS_C))):
        xs = [(x - t0) / 1e9 for x in win if x in recv[iface]]
        ax.plot(xs, [row] * len(xs), "|", ms=16, color=color)
        miss = [(x - t0) / 1e9 for x in win if x not in recv[iface]]
        ax.plot(miss, [row] * len(miss), "x", ms=7, color="#999")
    ax.set_yticks([0, 1], [WIRED, WIRELESS])
    ax.set_xlabel("elapsed (s), first two minutes")
    ax.set_ylim(-0.6, 1.6)
    single = sum(1 for n in lengths if n == 1)
    ax.set_title(
        f"x = did not arrive on {WIRELESS}: {sum(lengths)} frames in {len(lengths)} stretches, "
        f"{single} of them single"
    )
    fig.tight_layout()
    fig.savefig(OUT / "fig-loss.png", dpi=130)

    print("\nwrote fig-wired, fig-difference, fig-sawtooth, fig-wait-loss, fig-loss")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
