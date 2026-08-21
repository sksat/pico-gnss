#!/usr/bin/env python3
# /// script
# dependencies = ["matplotlib", "numpy", "pillow"]
# ///
"""Figures for the Pico 間リンクの NTP 時刻同期。

生データは repo top の `logs/20260820-pico-link/` から読む (gitignore 配下なのでコミットしない)。
出力はこのスクリプトの 1 つ上、`docs/report/pico-link-ntp/` に書く。

  uv run docs/report/pico-link-ntp/logs/plot_link.py
"""

import csv
import os
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.animation as animation  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.transforms as transforms  # noqa: E402
import numpy as np  # noqa: E402

# 図中に日本語を出すので、CJK を持つフォントを明示する。無いと豆腐になる。
matplotlib.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)
LOGS = os.path.join(HERE, "..", "..", "..", "..", "logs")
RAW = os.path.join(LOGS, "20260820-pico-link")
# 前回 (スイッチと AP を挟んで PC で受けた回) の生ログ。同じ数字を書き写さず、ここから読む。
RAW_MAIN = os.path.join(LOGS, "20260819-ntp-wired")


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
    """broadcast で同期しているあいだの時刻差。

    上段は GNSS 受信機の 1PPS を基準にした 2 枚それぞれの時刻差、下段は Pico client が Pico
    server をどれだけ再現したかで、後者が NTP そのものの精度にあたる。分けてあるのは別の量だから
    で、上段には Pico server 側の校正定数が乗り、下段では両方に等しく乗って落ちる。
    """
    server, client = read_pairs("pair-series.csv")
    shots = np.arange(len(server))
    diff = client - server

    fig, axes = plt.subplots(
        2,
        2,
        figsize=(11, 6),
        sharex="col",
        gridspec_kw={"width_ratios": [3, 1], "height_ratios": [1, 1]},
    )
    (ax, hx), (bx, gx) = axes

    ax.axhline(0, color="0.6", lw=0.8)
    ax.plot(
        shots, server, ".-", ms=3, lw=0.8, color="C0",
        label=f"Pico server {server.mean():+.2f} ± {server.std():.2f} µs",
    )
    ax.plot(
        shots, client, ".-", ms=3, lw=0.8, color="C1",
        label=f"Pico client {client.mean():+.2f} ± {client.std():.2f} µs",
    )
    ax.set_ylabel("受信機の 1PPS との\n時刻差 (µs)")
    ax.set_title("2 枚それぞれと GNSS 受信機", fontsize=10, loc="left")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(alpha=0.3)

    bx.axhline(0, color="0.6", lw=0.8)
    bx.plot(
        shots, diff, ".-", ms=3, lw=0.8, color="C3",
        label=f"{diff.mean():+.2f} ± {diff.std():.2f} µs",
    )
    bx.set_ylabel("Pico client −\nPico server (µs)")
    bx.set_xlabel("取り込み (約 5 秒に 1 回)")
    bx.set_title("NTP が渡せた精度", fontsize=10, loc="left")
    bx.legend(loc="upper left", fontsize=8)
    bx.grid(alpha=0.3)

    top = np.linspace(
        min(server.min(), client.min()), max(server.max(), client.max()), 22
    )
    hx.hist(server, bins=top, orientation="horizontal", alpha=0.65, color="C0")
    hx.hist(client, bins=top, orientation="horizontal", alpha=0.65, color="C1")
    gx.hist(
        diff,
        bins=np.linspace(diff.min(), diff.max(), 22),
        orientation="horizontal",
        alpha=0.75,
        color="C3",
    )
    for h, share in ((hx, ax), (gx, bx)):
        h.axhline(0, color="0.6", lw=0.8)
        h.set_ylim(share.get_ylim())
        h.tick_params(labelleft=False)
        h.grid(alpha=0.3)
    gx.set_xlabel("count")

    fig.suptitle(
        f"broadcast で同期しているあいだの時刻差 (n={len(server)}、約 8 分)", fontsize=11
    )
    fig.tight_layout()
    path = os.path.join(OUT, "fig-broadcast-series.png")
    fig.savefig(path, dpi=140)
    print(f"saved {path}")


# 1 回の取り込みで両方の 1PPS を読んだ run。server の絶対位置がばらついているのは狙って
# そうしたのではなく、UTC の基準を、PIO が捉えたエッジではなく FIFO から取り出した時刻で取っていて、
# 起動ごとに executor の起床順が変わるためである。ここではそれを逆に使う。
RUNS = [
    ("pair-series.csv", "broadcast"),
    ("pair-unicast2.csv", "unicast"),
    ("pair-pr10-retest.csv", "unicast"),
    ("pair-hwts.csv", "unicast"),
    ("pair-reviewfix.csv", "broadcast"),
    ("pair-offload.csv", "unicast"),
    ("pair-bisect.csv", "unicast"),
    ("pair-boot1.csv", "unicast"),
    ("pair-boot3.csv", "unicast"),
    ("pair-boot4.csv", "unicast"),
    ("pair-boot5.csv", "unicast"),
    ("pair-anchor-delay.csv", "unicast (時刻の基準を 100 µs 遅らせた)"),
]

# 垂直の設定は `scope_pair.py` が固定している (1 V/div、offset −1.5 V)。バイト 0-255 が
# 10 division にあたるので、真ん中のバイトが +1.5 V。
VOLTS_PER_BYTE = 10.0 / 255.0
CENTRE_VOLTS = 1.5


def read_trace(name):
    """`scope_pair.py trace` の CSV を frames[i][ch] = ボルトの配列 として読む。"""
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
            frames.setdefault(int(head), {})[int(ch)] = (
                b - 127.5
            ) * VOLTS_PER_BYTE + CENTRE_VOLTS
    return {k: v for k, v in frames.items() if len(v) == 3}, xinc


def _cross(w, falling):
    """しきい値を跨ぐ最初の点を、サンプルの小数位置で返す。"""
    lo, hi = w.min(), w.max()
    if hi - lo < 1.0:
        return None
    th = (lo + hi) / 2
    for i in range(1, len(w)):
        if falling and w[i - 1] >= th > w[i]:
            return (i - 1) + (w[i - 1] - th) / (w[i - 1] - w[i])
        if not falling and w[i - 1] < th <= w[i]:
            return (i - 1) + (th - w[i - 1]) / (w[i] - w[i - 1])
    return None


def _edges(frame, xinc):
    """トリガ (CH1 の立ち下がり) を 0 とした、CH3 と CH4 の立ち上がりの位置 (µs)。"""
    ref = _cross(frame[1], True)
    if ref is None:
        return None, None, None
    out = []
    for ch in (3, 4):
        e = _cross(frame[ch], False)
        out.append(None if e is None else (e - ref) * xinc / 1000.0)
    return ref, out[0], out[1]


# 並びも色もオシロの画面に合わせる。CH1 が上で、黄は白地で読めないので暗い金色にしてある。
LANES = [
    (1, 10.0, "#b8860b", "CH1  GNSS 受信機の 1PPS", "active low なので秒は立ち下がり"),
    (3, 5.0, "C4", "CH3  Pico server GP6", "GPS で規律した秒"),
    (4, 0.0, "C0", "CH4  Pico client GP6", "リンク越しの NTP だけで作った秒"),
]


def _draw_frame(ax, frame, xinc, ref, server_us, client_us):
    n = len(frame[1])
    t = (np.arange(n) - ref) * xinc / 1000.0
    # ラベルは軸の左端に貼る。データ座標だと拡大したときに画面の外へ出る。
    label_at = transforms.blended_transform_factory(ax.transAxes, ax.transData)
    for ch, base, colour, name, sub in LANES:
        ax.plot(t, frame[ch] + base, lw=1.2, color=colour)
        ax.text(0.012, base + 4.3, name, fontsize=9, color=colour, va="top",
                transform=label_at)
        ax.text(0.012, base + 2.9, sub, fontsize=7.5, color="0.35", va="top",
                transform=label_at)
    ax.axvline(0, color="0.35", lw=1.0, ls="--")
    ax.text(0, 14.6, "秒境界", fontsize=8.5, ha="center", color="0.25")

    for value, base, colour in ((server_us, 5.0, "C4"), (client_us, 0.0, "C0")):
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
            f"{value:+.2f} µs",
            fontsize=8.5,
            color=colour,
            ha="center",
        )
    ax.set_ylim(-1.2, 15.6)
    ax.set_yticks([])
    ax.set_xlabel("秒境界からの時間 (µs)")


def fig_scope_annotated(trace="trace-5us.csv"):
    """1 回の取り込みを、どの線が何なのかを書き入れて描き直したもの。"""
    frames, xinc = read_trace(trace)
    for i in sorted(frames):
        ref, s, c = _edges(frames[i], xinc)
        if ref is not None and s is not None and c is not None:
            break
    else:
        print(f"{trace}: 3 本とも読める取り込みが無い", file=sys.stderr)
        return
    ss = np.array([v for v in (_edges(frames[k], xinc)[1] for k in frames) if v is not None])
    cs = np.array([v for v in (_edges(frames[k], xinc)[2] for k in frames) if v is not None])
    fig, ax = plt.subplots(figsize=(9.5, 5.4))
    _draw_frame(ax, frames[i], xinc, ref, s, c)
    lim = max(9.0, abs(s) * 1.6, abs(c) * 1.6)
    ax.set_xlim(-lim, lim * 0.7)
    # 1 枚だけ見て「これがオフセットだ」と読まれないよう、run 全体の分布も表題に入れる。
    ax.set_title(
        "1 回の取り込み。3 本は同じトリガで、CH1 の立ち下がりが秒境界\n"
        f"この 1 枚は Pico server {s:+.2f} µs / Pico client {c:+.2f} µs、"
        f"同じ run {len(ss)} 回では {ss.mean():+.2f} ± {ss.std():.2f} µs / "
        f"{cs.mean():+.2f} ± {cs.std():.2f} µs",
        fontsize=9.5,
    )
    fig.tight_layout()
    path = os.path.join(OUT, "fig-scope-annotated.png")
    fig.savefig(path, dpi=140)
    print(f"saved {path}")
    print(f"  この 1 枚: server {s:+.2f} µs, client {c:+.2f} µs")
    print(
        f"  この run (n={len(ss)}): server {ss.mean():+.2f} µs (sd {ss.std():.2f}), "
        f"client {cs.mean():+.2f} µs (sd {cs.std():.2f})"
    )


def gif_scope(trace="trace-5us.csv"):
    """同じ描き方で連続した取り込みを並べる。1 shot では見えない揺れが見える。"""
    frames, xinc = read_trace(trace)
    usable = []
    for i in sorted(frames):
        ref, s, c = _edges(frames[i], xinc)
        if ref is not None:
            usable.append((frames[i], ref, s, c))
    if not usable:
        print(f"{trace}: 使える取り込みが無い", file=sys.stderr)
        return

    # GIF は 1 コマが 1 枚の PNG になるので、静止画より小さく粗く作る。
    fig, (ax, tx) = plt.subplots(
        2, 1, figsize=(8.4, 6.0), dpi=80, gridspec_kw={"height_ratios": [3, 1.1]}
    )
    ss = [s for _, _, s, _ in usable]
    cs = [c for _, _, _, c in usable]
    seen = [v for v in ss + cs if v is not None]
    lim = max(abs(v) for v in seen) * 1.3 if seen else 10.0

    # コマごとに tight_layout を呼ぶと表題が切れたり枠が揺れたりするので、一度だけ決める。
    fig.subplots_adjust(left=0.07, right=0.98, top=0.92, bottom=0.09, hspace=0.42)

    def render(k):
        ax.clear()
        tx.clear()
        frame, ref, s, c = usable[k]
        _draw_frame(ax, frame, xinc, ref, s, c)
        ax.set_xlim(-lim, lim * 0.7)
        ax.set_title(f"取り込み {k + 1}/{len(usable)}", fontsize=10)
        tx.axhline(0, color="0.6", lw=0.8)
        tx.plot(range(k + 1), ss[: k + 1], ".-", ms=3, lw=0.8, color="C4",
                label="Pico server")
        tx.plot(range(k + 1), cs[: k + 1], ".-", ms=3, lw=0.8, color="C0",
                label="Pico client")
        tx.set_xlim(-1, len(usable))
        tx.set_ylim(-lim, lim * 0.7)
        tx.set_ylabel("秒境界からのずれ (µs)", fontsize=8)
        tx.set_xlabel("取り込み", fontsize=8)
        tx.tick_params(labelsize=7)
        tx.legend(loc="upper right", fontsize=7, ncol=2)
        tx.grid(alpha=0.3)

    anim = animation.FuncAnimation(fig, render, frames=len(usable), interval=200)
    path = os.path.join(OUT, "fig-scope.gif")
    anim.save(path, writer=animation.PillowWriter(fps=5))
    print(f"saved {path} ({len(usable)} frames, {os.path.getsize(path) / 1e6:.1f} MB)")


def fig_ntp_accuracy():
    """NTP が Pico server の秒をどれだけ忠実に Pico client へ渡せたか。

    見るのは client GP6 − server GP6 で、1 回の取り込みから両方読んだ差である。受信機を基準に
    した絶対位置と違い、server 側の校正定数は両方に等しく乗るのでこの差からは落ちる。
    """
    runs = []
    for name, label in RUNS:
        if not os.path.exists(os.path.join(RAW, name)):
            continue
        server, client = read_pairs(name)
        if len(server) == 0:
            continue
        runs.append((label, server, client - server))
    runs.sort(key=lambda r: r[1].mean())

    fig, (ax, bx) = plt.subplots(
        1, 2, figsize=(12, 5.0), gridspec_kw={"width_ratios": [1.1, 1]}
    )
    band = 10.0
    rng = np.random.default_rng(0)
    for i, (label, server, diff) in enumerate(runs):
        ax.plot(
            diff,
            i + rng.uniform(-0.16, 0.16, len(diff)),
            ".",
            ms=3.5,
            alpha=0.45,
            color=f"C{i % 10}",
        )
        ax.errorbar(
            diff.mean(),
            i,
            xerr=diff.std(),
            fmt="o",
            ms=5,
            capsize=3,
            color=f"C{i % 10}",
            zorder=3,
        )
        bx.plot(server, diff, ".", ms=4, alpha=0.6, color=f"C{i % 10}")
    ax.axvspan(-band, band, color="0.88", zorder=0)
    ax.axvline(0, color="0.5", lw=0.8, zorder=1)
    ax.set_yticks(range(len(runs)))
    ax.set_yticklabels(
        [f"{lab}\nPico server {srv.mean():+.1f} µs (n={len(d)})" for lab, srv, d in runs],
        fontsize=7.5,
    )
    ax.set_xlabel("Pico client GP6 − Pico server GP6 (µs)")
    ax.set_title("Pico client が Pico server の秒を再現した精度", fontsize=10)
    ax.grid(axis="x", alpha=0.3)

    bx.axhspan(-band, band, color="0.88", zorder=0)
    bx.axhline(0, color="0.5", lw=0.8, zorder=1)
    bx.set_xlabel("Pico server GP6 − 受信機の秒 (µs)")
    bx.set_ylabel("Pico client GP6 − Pico server GP6 (µs)")
    bx.set_title("Pico server が受信機からどれだけずれていても変わらない", fontsize=10)
    bx.grid(alpha=0.3)

    allo = np.concatenate([d for _, _, d in runs])
    lo = min(s.mean() for _, s, _ in runs)
    hi = max(s.mean() for _, s, _ in runs)
    dlo = min(d.mean() for _, _, d in runs)
    dhi = max(d.mean() for _, _, d in runs)
    sd_lo = min(d.std() for _, _, d in runs)
    sd_hi = max(d.std() for _, _, d in runs)
    fig.suptitle(
        f"取り込み {len(allo)} 回。Pico server の位置が {lo:+.1f} µs から {hi:+.1f} µs まで"
        f"動いても、Pico client は Pico server に対して run ごとの平均 {dlo:+.1f}〜{dhi:+.1f} µs "
        f"(run 内の sd {sd_lo:.1f}〜{sd_hi:.1f} µs)",
        fontsize=10,
    )
    fig.tight_layout()
    path = os.path.join(OUT, "fig-ntp-accuracy.png")
    fig.savefig(path, dpi=140)
    print(f"saved {path}")


RECV_LINE = re.compile(r"^(?P<iface>\S+)\s+\S+\s+mode=\S+.*?offset=(?P<off>[-+\d.]+)us")
# 前回の受信ホストのインタフェース名。有線と無線で同じ broadcast が二度届く。
MAIN_IFACES = [
    ("wlp1s0", "broadcast・無線 (AP 経由)"),
    ("eth0", "broadcast・有線 (switch 直)"),
]
# 前回の記録のうち、送信時刻の補正を入れたあとのもの。構成どうしを比べるならこちらを使う。
MAIN_LOG = "paths-corrected.log"


def read_main_offsets(name=MAIN_LOG):
    """前回の受信ログから、インタフェースごとの offset (µs) を読む。

    ログの `offset` は「パケットが申告した送信時刻 − カーネルが付けた受信時刻」である。
    片道の所要時間も、受信ホストの時計のずれも、この 1 つの値に入っていて分けられない。
    """
    out = {}
    with open(os.path.join(RAW_MAIN, name)) as f:
        for line in f:
            m = RECV_LINE.match(line)
            if m:
                out.setdefault(m.group("iface"), []).append(float(m.group("off")))
    return {k: np.array(v) for k, v in out.items()}


def read_mode_diff(tag):
    """`pair-mode-<tag>.csv` の client − server を µs で読む。

    オシロが 1 回のトリガで読んだ、2 枚の GP6 の立ち上がりの差である。PC の時計は入らない。
    """
    server, client = read_pairs(f"pair-mode-{tag}.csv")
    return client - server


def fig_progression():
    """同じ broadcast のまま、構成を変えると桁がどう動くか。

    どの行も「その run の全サンプルの相加平均」と「その標準偏差」だが、平均している量は上下で
    違う。上は PC が読んだ「申告 − 受信」、下はオシロが読んだ「client の秒 − server の秒」で、
    前者には片道の所要時間と PC の時計が入っていて分けられない。比べているのは桁である。
    """
    main = read_main_offsets()
    rows = []
    for iface, label in MAIN_IFACES:
        if iface in main:
            rows.append(
                ("これまで: PC で受ける", label, "申告 − PC の受信", main[iface])
            )
    for tag, label in (
        ("broadcast", "broadcast・片道"),
        ("unicast", "unicast・往復"),
    ):
        if os.path.exists(os.path.join(RAW, f"pair-mode-{tag}.csv")):
            rows.append(
                (
                    "今回: Pico 直結",
                    label,
                    "client の秒 − server の秒",
                    read_mode_diff(tag),
                )
            )
    if not rows:
        print("progression: 読める記録が無い", file=sys.stderr)
        return

    fig, ax = plt.subplots(figsize=(11, 4.4))
    for i, (era, label, _quantity, v) in enumerate(rows):
        y = len(rows) - 1 - i
        colour = "#c1442e" if era.startswith("これまで") else "#2e7d5b"
        mag = abs(v.mean())
        ax.errorbar(
            mag,
            y,
            xerr=[[min(v.std(), mag * 0.95)], [v.std()]],
            fmt="o",
            ms=7,
            capsize=4,
            lw=1.6,
            color=colour,
        )
        # 端の行は注釈が枠から出るので、点の左右どちらに置くかを位置で決める。
        right = mag > 1e4
        ax.annotate(
            f"{fmt_us(v.mean())} ± {fmt_us(v.std(), sign=False)}  (n={len(v)})",
            xy=(mag, y),
            xytext=(-12 if right else 12, 11),
            textcoords="offset points",
            fontsize=9,
            ha="right" if right else "left",
            color=colour,
        )

    ax.set_xscale("log")
    ax.set_xlim(4, 6e5)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels(
        [f"{era}\n{label}\n{quantity}" for era, label, quantity, _ in reversed(rows)],
        fontsize=8.5,
    )
    ax.set_ylim(-0.9, len(rows) - 0.3)
    ax.set_xlabel("受け取った側が server の秒からどれだけずれていたか — 平均の大きさ (µs、対数)")
    ax.set_title(
        "同じ broadcast のまま、無線と PC の時計が測定系から外れた分だけ縮む", fontsize=10
    )
    ax.grid(axis="x", which="both", alpha=0.3)
    fig.text(
        0.5,
        0.015,
        "どの行も、その run の全サンプルの相加平均と標準偏差である (点が平均の大きさ、"
        "横棒が標準偏差)。ただし平均している量が上下で違う。\n"
        "上 2 行の「申告 − PC の受信」には、片道の所要時間と PC の時計のずれが入っていて"
        "分けられない。下 2 行にはどちらも入らない。同じ量ではないので、比べているのは桁である。\n"
        "下 2 行どうしも、平均は比べられない (送信側の校正定数が配り方ごとに違う)。"
        "比べられるのは横棒の長さ、つまり 1 回ごとの揺れのほうである。",
        fontsize=7.8,
        color="0.35",
        ha="center",
    )
    fig.tight_layout(rect=(0, 0.14, 1, 1))
    path = os.path.join(OUT, "fig-progression.png")
    fig.savefig(path, dpi=140)
    print(f"saved {path}")


def fmt_us(value, sign=True):
    """µs の値を、桁に合わせて µs か ms で書く。"""
    unit, scale = ("ms", 1000.0) if abs(value) >= 1000.0 else ("µs", 1.0)
    return f"{value / scale:{'+' if sign else ''}.2f} {unit}"


def main():
    if not os.path.isdir(RAW):
        print(f"生データが見つからない: {RAW}", file=sys.stderr)
        raise SystemExit(1)
    if os.path.exists(os.path.join(RAW_MAIN, "paths.log")):
        fig_progression()
    else:
        print(f"前回の生ログが無いので進み方の図は飛ばす: {RAW_MAIN}", file=sys.stderr)
    fig_broadcast_series()
    fig_ntp_accuracy()
    if os.path.exists(os.path.join(RAW, "trace-5us.csv")):
        fig_scope_annotated()
        gif_scope()
    else:
        print("trace-5us.csv が無いので波形の図は飛ばす", file=sys.stderr)


if __name__ == "__main__":
    main()
