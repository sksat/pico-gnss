#!/usr/bin/env python3
# /// script
# dependencies = ["matplotlib", "numpy", "pillow"]
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
import matplotlib.animation as animation  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.transforms as transforms  # noqa: E402
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


# 波形を連続で取った回。1 コマが 1 取り込みで、GIF はこれを並べる。
TRACES = [
    ("NTP 駆動", "pair-gif2-ntp-trace.csv", "tab:red"),
    ("PTP 駆動", "pair-gif3-ptp-trace.csv", "tab:green"),
]

# `measure_pair.py` が固定している垂直の設定。バイト 0-255 が 10 division にあたる。
VOLTS_PER_BYTE = 10.0 / 255.0
CENTRE_VOLTS = 1.5

# 画面に並べる 3 本。オシロと同じ順に、上から。
LANES = [
    (1, 10.0, "#b8860b", "CH1  GNSS 受信機の 1PPS", "active low なので秒は立ち下がり"),
    (3, 5.0, "tab:purple", "CH3  Pico server の GP6", "GPS で規律した秒"),
    (4, 0.0, "tab:blue", "CH4  Pico client の GP6", "リンク越しに渡された秒"),
]


def read_trace(name):
    """`measure_pair.py trace` の CSV を frames[i][ch] = ボルトの配列 として読む。"""
    frames, xinc = {}, None
    with open(os.path.join(RAW, name)) as f:
        for line in f:
            if line.startswith("#"):
                if "xinc_ns=" in line:
                    xinc = float(line.split("xinc_ns=")[1].split()[0])
                continue
            head, _, rest = line.partition(",")
            ch, _, samples = rest.partition(",")
            try:
                b = np.array([int(v) for v in samples.split(",")], dtype=float)
            except ValueError:
                continue  # 書き込み途中の最終行
            if len(b) < 100:
                continue
            # 生のバイトのまま持つ。電圧に直すのは描くときだけで、エッジは
            # `measure_pair.py` と同じバイトのしきい値で取る — 別の取り方をすると、
            # 図の矢印と本文の数値が食い違う。
            frames.setdefault(int(head), {})[int(ch)] = b
    return frames, xinc


# バイト 0-255 が 10 division にあたるので、その真ん中がしきい値である。
# **全チャネル共通**にする。チャネルごとに自分の min/max の中点を取ると、オーバーシュートの
# 違いだけで交差点が数十 ns 動き、2 本の矢印の長さが互いに比べられなくなる。
CROSS_LEVEL = 127.0


def _cross(v, falling, level=CROSS_LEVEL):
    """立ち上がり (または立ち下がり) が `level` を渡る点を、サンプル間で内挿して返す。"""
    lo, hi = float(np.min(v)), float(np.max(v))
    if hi - lo < 20:  # 平ら: この channel にエッジは無い
        return None
    for i in range(1, len(v)):
        if falling and v[i - 1] >= level > v[i]:
            return i - 1 + (v[i - 1] - level) / (v[i - 1] - v[i])
        if not falling and v[i - 1] < level <= v[i]:
            return i - 1 + (level - v[i - 1]) / (v[i] - v[i - 1])
    return None


def _edges(frame, xinc):
    """受信機の秒 (CH1 の立ち下がり) を 0 とした、2 枚の GP6 の位置 (µs)。"""
    ref = _cross(frame[1], True)
    if ref is None:
        return None, None, None
    out = []
    for ch in (3, 4):
        e = _cross(frame[ch], False)
        out.append(None if e is None else (e - ref) * xinc / 1000.0)
    return ref, out[0], out[1]


def _draw(ax, frame, xinc, ref, server_us, client_us):
    n = len(frame[1])
    x = (np.arange(n) - ref) * xinc / 1000.0
    at = transforms.blended_transform_factory(ax.transAxes, ax.transData)
    for ch, base, colour, name, sub in LANES:
        ax.plot(x, frame[ch] * VOLTS_PER_BYTE - CENTRE_VOLTS + base, lw=1.2, color=colour)
        ax.text(0.012, base + 4.3, name, fontsize=9, color=colour, va="top", transform=at)
        ax.text(0.012, base + 2.9, sub, fontsize=7.5, color="0.35", va="top", transform=at)
    ax.axvline(0, color="0.35", lw=1.0, ls="--")
    ax.text(0, 14.6, "秒境界", fontsize=8.5, ha="center", color="0.25")
    for value, base, colour in ((server_us, 5.0, "tab:purple"), (client_us, 0.0, "tab:blue")):
        if value is None:
            continue
        ax.annotate(
            "",
            xy=(value, base + 1.6),
            xytext=(0, base + 1.6),
            arrowprops=dict(arrowstyle="<->", color=colour, lw=1.0),
        )
        ax.text(
            value / 2,
            base + 1.8,
            f"{value * 1000:+.0f} ns",
            fontsize=8.5,
            color=colour,
            ha="center",
        )
    ax.set_ylim(-1.2, 15.6)
    ax.set_yticks([])
    ax.set_xlabel("秒境界からの時間 (µs)")


def check_consistent(name, usable, series):
    """図に出す数字が、同じ 1 つの取り込みから出ていることを確かめる。

    一度やった間違いなので、繰り返さないように自動で見る: 上のパネルの矢印は
    `usable[k]` から、下のパネルの折れ線は別に組み立てた列から描いているので、両者が同じ
    取り込みを指しているかはコードを読まないと分からない。ずれても絵は破綻せず、値だけが
    静かに食い違う。

    `series` は `(名前, 列, usable から同じ値を取り出す関数)` の並び。
    """
    for label, seq, of in series:
        assert len(seq) == len(usable), f"{name}: {label} の長さが取り込み数と違う"
        for k in (0, len(usable) // 2, len(usable) - 1):
            a, b = seq[k], of(usable[k])
            assert abs(a - b) < 1e-9, f"{name}: {label} の {k} 番目が矢印と食い違う ({a} vs {b})"


def gif_pair(label, name, colour, out_name):
    """1 コマ 1 取り込みで並べる。1 枚では見えない揺れが見える。"""
    frames, xinc = read_trace(name)
    usable = []
    for i in sorted(frames):
        if len(frames[i]) < 3:
            continue
        ref, s, c = _edges(frames[i], xinc)
        if ref is not None and s is not None and c is not None:
            usable.append((frames[i], ref, s, c))
    if not usable:
        print(f"{name}: 使える取り込みが無い", file=sys.stderr)
        return

    fig, (ax, tx) = plt.subplots(
        2, 1, figsize=(8.4, 6.0), dpi=80, gridspec_kw={"height_ratios": [3, 1.1]}
    )
    ss = [s for _, _, s, _ in usable]
    cs = [c for _, _, _, c in usable]

    check_consistent(
        out_name,
        usable,
        [
            ("server の矢印", ss, lambda u: u[2]),
            ("client の矢印", cs, lambda u: u[3]),
        ],
    )
    diff = [c - s for s, c in zip(ss, cs)]
    seen = ss + cs
    lo, hi = min(seen), max(seen)
    pad = (hi - lo) * 0.25 + 0.2
    # コマごとに tight_layout を呼ぶと枠が揺れるので、一度だけ決める。
    fig.subplots_adjust(left=0.11, right=0.98, top=0.90, bottom=0.09, hspace=0.42)
    # 下のパネルは上の矢印 2 本と同じ量を描くので、範囲も両方から取る。
    both = ss + cs
    lo_d, hi_d = min(both) * 1000, max(both) * 1000
    margin = (hi_d - lo_d) * 0.15 + 20

    def render(k):
        ax.clear()
        tx.clear()
        frame, ref, s, c = usable[k]
        _draw(ax, frame, xinc, ref, s, c)
        ax.set_xlim(lo - pad, hi + pad)
        ax.set_title(
            f"{label}   取り込み {k + 1}/{len(usable)}   "
            f"client − server {(c - s) * 1000:+.0f} ns",
            fontsize=10,
        )
        tx.axhline(0, color="0.6", lw=0.8)
        # 上の 2 本の矢印と同じ量を描く。ここに client − server を描くと、画面の矢印
        # (それぞれが GPS の秒からどれだけ離れているか) と数字が合わず、読み手が突き合わせられない。
        tx.plot(range(k + 1), [v * 1000 for v in ss[: k + 1]], ".-", ms=3, lw=0.8,
                color="tab:purple", label="server")
        tx.plot(range(k + 1), [v * 1000 for v in cs[: k + 1]], ".-", ms=3, lw=0.8,
                color="tab:blue", label="client")
        tx.set_xlim(-1, len(usable))
        # データの範囲から取る。0 を基準に取ると、offset が 0 から離れた run で推移が枠外に出る。
        tx.set_ylim(lo_d - margin, hi_d + margin)
        tx.set_ylabel("受信機の秒からのずれ (ns)", fontsize=8)
        tx.legend(fontsize=7, ncol=2, loc="upper right")
        tx.set_xlabel("取り込み", fontsize=8)
        tx.tick_params(labelsize=7)
        tx.grid(alpha=0.3)

    anim = animation.FuncAnimation(fig, render, frames=len(usable), interval=200)
    path = os.path.join(OUT, out_name)
    anim.save(path, writer=animation.PillowWriter(fps=5))
    plt.close(fig)
    print(
        f"saved {path} ({len(usable)} frames, {os.path.getsize(path) / 1e6:.1f} MB)  "
        f"mean {st.mean(diff) * 1000:+.1f} ns  sd {st.pstdev(diff) * 1000:.1f}"
    )


def _draw_two(ax, frame, xinc, ref, diff_us):
    """server と client の GP6 だけを、server の立ち上がりを 0 にして描く。

    受信機の 1PPS を外し、基準を server 側に置き換えたもの。3 本のほうは「それぞれが GPS の秒
    からどれだけ離れているか」を見る図で、そこには両方の板の出力チェーンのオフセットが乗って
    いる。ここで見たいのは **2 枚がどれだけ揃っているか** だけなので、共通に乗るものは基準ごと
    落とす。
    """
    n = len(frame[3])
    x = (np.arange(n) - ref) * xinc / 1000.0
    at = transforms.blended_transform_factory(ax.transAxes, ax.transData)
    for ch, base, colour, name in (
        (3, 5.0, "tab:purple", "CH3  Pico server の GP6  (基準)"),
        (4, 0.0, "tab:blue", "CH4  Pico client の GP6"),
    ):
        ax.plot(x, frame[ch] * VOLTS_PER_BYTE - CENTRE_VOLTS + base, lw=1.4, color=colour)
        ax.text(0.012, base + 4.3, name, fontsize=9.5, color=colour, va="top", transform=at)
    ax.axvline(0, color="tab:purple", lw=1.0, ls="--")
    ax.annotate(
        "",
        xy=(diff_us, 1.6),
        xytext=(0, 1.6),
        arrowprops=dict(arrowstyle="<->", color="tab:blue", lw=1.2),
    )
    ax.text(
        diff_us / 2,
        1.9,
        f"{diff_us * 1000:+.0f} ns",
        fontsize=10,
        color="tab:blue",
        ha="center",
    )
    ax.set_ylim(-1.2, 10.6)
    ax.set_yticks([])
    ax.set_xlabel("Pico server の秒からの時間 (µs)")


def gif_two(label, name, colour, out_name):
    """server を基準にして client だけを見る。2 枚がどれだけ揃っているか。"""
    frames, xinc = read_trace(name)
    usable = []
    for i in sorted(frames):
        if len(frames[i]) < 3:
            continue
        ref = _cross(frames[i][3], False)
        c = _cross(frames[i][4], False)
        if ref is None or c is None:
            continue
        usable.append((frames[i], ref, (c - ref) * xinc / 1000.0))
    if not usable:
        print(f"{name}: 使える取り込みが無い", file=sys.stderr)
        return

    diff = [d for _, _, d in usable]
    lo, hi = min(diff + [0.0]), max(diff + [0.0])
    pad = (hi - lo) * 0.35 + 0.15
    span = max(abs(min(diff)), abs(max(diff))) * 1000

    check_consistent(out_name, usable, [("矢印", diff, lambda u: u[2])])

    fig, (ax, tx) = plt.subplots(
        2, 1, figsize=(8.4, 5.4), dpi=80, gridspec_kw={"height_ratios": [2.6, 1.1]}
    )
    fig.subplots_adjust(left=0.11, right=0.98, top=0.90, bottom=0.10, hspace=0.45)

    def render(k):
        ax.clear()
        tx.clear()
        frame, ref, d = usable[k]
        _draw_two(ax, frame, xinc, ref, d)
        ax.set_xlim(lo - pad, hi + pad)
        ax.set_title(
            f"{label}   取り込み {k + 1}/{len(usable)}   client − server {d * 1000:+.0f} ns",
            fontsize=10.5,
        )
        tx.axhline(st.mean(diff) * 1000, color="0.6", lw=0.8, ls="--")
        tx.plot(range(k + 1), [v * 1000 for v in diff[: k + 1]], ".-", ms=3, lw=0.8, color=colour)
        tx.set_xlim(-1, len(usable))
        tx.set_ylim(min(diff) * 1000 - span * 0.12 - 20, max(diff) * 1000 + span * 0.12 + 20)
        tx.set_ylabel("client − server (ns)", fontsize=8)
        tx.set_xlabel("取り込み", fontsize=8)
        tx.tick_params(labelsize=7)
        tx.grid(alpha=0.3)

    anim = animation.FuncAnimation(fig, render, frames=len(usable), interval=200)
    path = os.path.join(OUT, out_name)
    anim.save(path, writer=animation.PillowWriter(fps=5))
    plt.close(fig)
    print(
        f"saved {path} ({len(usable)} frames, {os.path.getsize(path) / 1e6:.1f} MB)  "
        f"mean {st.mean(diff) * 1000:+.1f} ns  sd {st.pstdev(diff) * 1000:.1f}"
    )


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

    for label, name, colour in TRACES:
        if os.path.exists(os.path.join(RAW, name)):
            which = "ntp" if "ntp" in name else "ptp"
            gif_pair(label, name, colour, f"fig-scope-{which}.gif")
            gif_two(label, name, colour, f"fig-pair-{which}.gif")
        else:
            print(f"{name} が無いので GIF は飛ばす", file=sys.stderr)


if __name__ == "__main__":
    main()
