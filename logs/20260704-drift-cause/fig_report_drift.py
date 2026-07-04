#!/usr/bin/env python3
"""残った実ドリフト節のレポート用図 4 枚。
fig-inject:       出力へ +1ppb 注入してもピンの歩きは折れない (作る側の消去)
fig-shadow-march: 測るだけの基準ズレの階段が、ピンの歩きと重なる (基準の直接観測)
fig-drift-elim:   左: ピンが歩く最中も同じピンを見る 2 本は離れない / 右: 測る間隔 4 倍でも傾き不変
fig-w900:         low の時間を揃えると歩きが消える (介入)
usage: uv run --with matplotlib python3 logs/20260704-drift-cause/fig_report_drift.py
"""
import os
import re
import statistics as st
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",):
    if os.path.exists(fp):
        try: font_manager.fontManager.addfont(fp)
        except Exception: pass
plt.rcParams["font.family"] = "Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"] = False
HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.dirname(HERE)
OUT = os.path.join(os.path.dirname(os.path.dirname(HERE)),
                   "docs", "report", "precision-ladder", "precision-figs")
KEXP = re.compile(r"KEXP count=(\d+) gen=\d+ c0=(\d+) c2=(\d+) c3=(\d+) c3n=(\d+)")
KSH3 = re.compile(r"KSH3 count=(\d+) on=(\d)")
GREEN, DGREEN, RED, BROWN = "#1a7a1a", "#145914", "#cc3333", "#8a5a00"


def f32(u):
    u &= 0xFFFFFFFF
    return u - (1 << 32) if u >= (1 << 31) else u


def slope(xs, ys):
    n = len(xs); mx = sum(xs) / n; my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx else float("nan")


def load_hwphase(path, want=("hwphase_ns",)):
    out = []
    for ln in open(path, errors="replace"):
        if "PPSGEN count=" not in ln:
            continue
        m = re.match(r"^(\d+\.\d+)\s", ln)
        if not m:
            continue
        d = dict(re.findall(r"(\w+)=(-?\d+)", ln))
        if "hwphase_ns" in d and abs(int(d["hwphase_ns"])) < 2000:
            out.append((float(m.group(1)), int(d["hwphase_ns"]), int(d.get("count", 0))))
    return out


def shots(path):
    return [(float(a), float(b)) for a, b in (ln.split() for ln in open(path))]


def gap_dense(scope, rtt_w, tol=2.5):
    gx, gy, j = [], [], 0
    for t, v in scope:
        while j + 1 < len(rtt_w) and abs(rtt_w[j + 1][0] - t) <= abs(rtt_w[j][0] - t):
            j += 1
        if abs(rtt_w[j][0] - t) <= tol:
            gx.append(t); gy.append(v - rtt_w[j][1])
    return gx, gy


def gap_auto_series(rttpath, shotpath, nmin=300):
    hw = [(t, h) for t, h, _ in load_hwphase(rttpath)]
    sc = shots(shotpath)
    best = None
    for k in range(0, 1200, 2):
        bw = sc[0][0] - k
        gx, gy = gap_dense(sc, [(bw + t, h) for t, h in hw])
        if len(gy) < nmin:
            continue
        v = st.pvariance(gy)
        if best is None or v < best[0]:
            best = (v, sc[0][0] - k)
    boot = best[1]
    gx, gy = gap_dense(sc, [(boot + t, h) for t, h in hw])
    gx = [t - boot for t in gx]
    med = st.median(gy)
    kp = [i for i in range(len(gy)) if abs(gy[i] - med) < 300]
    gx = [gx[i] for i in kp]; gy = [gy[i] for i in kp]
    return gx, gy  # rtt 相対秒, ns


def runmean(y, w=60):
    return [sum(y[max(0, i - w + 1):i + 1]) / (i + 1 - max(0, i - w + 1)) for i in range(len(y))]


# ================= fig-inject (上段: 模式図 / 下段: 実測) =================
def fig_inject():
    from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle
    rtt = load_hwphase(os.path.join(HERE, "inj-rtt.log"))
    t_inj = min((t for t, h, c in rtt if c >= 300))
    gx, gy = gap_auto_series(os.path.join(HERE, "inj-rtt.log"), os.path.join(HERE, "inj-dense.shots"))
    t0 = gx[0]
    xm = [(x - t0) / 60 for x in gx]
    tim = (t_inj - t0) / 60
    base = [(x, y) for x, y in zip(xm, gy) if x < tim - 0.3]
    sb = slope([p[0] for p in base], [p[1] for p in base])
    ib = sum(p[1] for p in base) / len(base) - sb * (sum(p[0] for p in base) / len(base))
    y_inj = ib + sb * tim
    xend = max(xm)
    y0 = [g - y_inj for g in gy]

    fig = plt.figure(figsize=(10.2, 8.6))
    gs = fig.add_gridspec(2, 1, height_ratios=[1.0, 0.85], hspace=0.18)

    # ---- 上段: 上乗せの場所の模式図 ----
    ax = fig.add_subplot(gs[0]); ax.set_xlim(0, 13); ax.set_ylim(0, 7.2); ax.axis("off")

    C_CPU, C_CPU_E = "#eef2f8", "#446"     # firmware (CPU)
    C_PIO, C_PIO_E = "#eaf4ea", "#22661f"  # PIO
    C_PIN, C_PIN_E = "#f7f7f7", "#666"     # ピン

    def dbox(x, y, w, h, label, sub=None, ec=C_CPU_E, fc=C_CPU, fs=9.6):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.06", fc=fc, ec=ec, lw=1.3, zorder=4))
        if sub:
            ax.text(x + w / 2, y + h * 0.63, label, ha="center", va="center", fontsize=fs, zorder=5)
            ax.text(x + w / 2, y + h * 0.27, sub, ha="center", va="center", fontsize=7.6, color="#555", zorder=5)
        else:
            ax.text(x + w / 2, y + h / 2, label, ha="center", va="center", fontsize=fs, zorder=5)

    def darrow(p1, p2, color="#446", lw=1.5, rad=0.0):
        ax.add_patch(FancyArrowPatch(p1, p2, arrowstyle="->", color=color, lw=lw,
                                     connectionstyle=f"arc3,rad={rad}", zorder=3, shrinkA=2, shrinkB=2))

    ax.add_patch(FancyBboxPatch((0.25, 0.9), 11.0, 4.8, boxstyle="round,pad=0.12",
                                fc="none", ec="#888", lw=1.3, ls=(0, (5, 4)), zorder=1))
    ax.text(5.75, 0.42, "破線の内側 = 制御ループが見張っている範囲。ここで入った誤差は一周して読みに現れ、補正で打ち消される",
            fontsize=8.8, color="#555", ha="center", zorder=6)

    Y = 4.35
    dbox(0.55, Y - 0.5, 2.3, 1.35, "制御ループ", "ずれを見て毎秒\n周期を決める")
    cx, cy = 3.75, Y + 0.18
    ax.add_patch(Circle((cx, cy), 0.28, fc="#fff", ec="#cc3333", lw=1.6, zorder=5))
    ax.text(cx, cy, "+", ha="center", va="center", fontsize=15, color="#cc3333", zorder=6)
    dbox(4.55, Y - 0.5, 2.35, 1.35, "出力の生成", "受け取った周期どおりに\nエッジを立てる", ec=C_PIO_E, fc=C_PIO)
    dbox(7.75, Y - 0.5, 1.85, 1.35, "出力ピン (GP3)", None, ec=C_PIN_E, fc=C_PIN)
    darrow((2.9, cy), (3.45, cy))
    ax.text(3.15, cy - 0.62, "周期の指定", ha="center", fontsize=8.0, color="#446", zorder=6)
    darrow((4.05, cy), (4.5, cy))
    darrow((6.95, cy), (7.7, cy))

    ax.text(cx, 6.75, "+1 ppb 上乗せ (毎秒 1 ns ぶん)", ha="center", fontsize=9.6, color="#cc3333", weight="bold")
    darrow((cx, 6.5), (cx, cy + 0.32), color="#cc3333", lw=1.7)

    # 戻り: 出力ピン → (配線) → loopback 入力ピン → 捕捉カウンタ → 制御ループ
    dbox(7.75, 1.3, 1.85, 1.35, "loopback\n入力ピン (GP4)", None, ec=C_PIN_E, fc=C_PIN, fs=8.8)
    darrow((8.67, Y - 0.55), (8.67, 2.72), color="#666", lw=1.5)
    ax.text(9.0, 3.35, "配線で 1 本戻す", ha="left", fontsize=8.2, color="#666")
    dbox(3.9, 1.3, 3.1, 1.35, "loopback の捕捉カウンタ", "ピンのエッジを捕まえる", ec=C_PIO_E, fc=C_PIO)
    darrow((7.7, 1.97), (7.05, 1.97), color=C_PIO_E, lw=1.5)
    darrow((3.85, 2.1), (1.7, Y - 0.52), color=C_PIO_E, lw=1.6, rad=0.0)
    ax.text(5.35, 3.15, "上乗せの分も実物のエッジとして読みに入る", ha="center", fontsize=8.2, color=C_PIO_E)
    ax.text(1.95, 2.0, "位相誤差の増加として届く\n→ 周波数補正で打ち消す", ha="center", fontsize=8.2, color=C_PIO_E)

    dbox(11.6, Y - 0.5, 1.25, 1.35, "オシロ", "GPS-R の\nエッジと比較", fc="#fdf6e3", ec="#b8860b", fs=9.0)
    darrow((9.65, cy), (11.55, cy), color="#b8860b", lw=1.5)
    ax.text(12.2, Y - 1.5, "独立観測。\n打ち消されず残った\nずれだけが見える", ha="center", fontsize=8.0, color="#8a6a0b")

    # ---- 下段: 実測 ----
    ax2 = fig.add_subplot(gs[1])
    ax2.axvline(tim, color=RED, ls="--", lw=1.3)
    ax2.plot(xm, y0, ".", ms=2, alpha=0.25, color=GREEN)
    ax2.plot(xm, runmean(y0, 45), "-", color=DGREEN, lw=2,
             label="ピンの上のずれ − 内部位相 (オシロ実測)")
    ax2.plot([tim, xend], [0, (sb + 60) * (xend - tim)], "--", color=RED, lw=1.8,
             label="上乗せが打ち消されずピンに出た場合 (+60 ns/min)")
    ax2.annotate("ここから +1 ppb を上乗せ", xy=(tim, 0), xytext=(tim + 3.4, 60),
                 fontsize=9.5, color=RED, ha="center",
                 arrowprops=dict(arrowstyle="->", color=RED, lw=1.1))
    ax2.annotate("実測は変わらない = 制御ループが打ち消した", xy=(xend - 2, runmean(y0, 45)[-1]),
                 xytext=(tim + 6.5, 210), fontsize=9.5, color=DGREEN,
                 arrowprops=dict(arrowstyle="->", color=DGREEN, lw=1.1))
    ax2.set_ylim(min(y0) - 20, (sb + 60) * (xend - tim) * 0.5)
    ax2.set_xlabel("経過 [min]")
    ax2.set_ylabel("ピンの上のずれ [ns, 上乗せ時点を 0 に]")
    ax2.grid(ls=":", alpha=0.4); ax2.legend(loc="upper left", fontsize=9)
    fig.savefig(os.path.join(OUT, "fig-inject.png"), dpi=110, bbox_inches="tight"); plt.close(fig)
    print(f"fig-inject: baseline {sb:+.1f} ns/min")


# ================= fig-shadow-march (上段: 模式図 / 下段: 実測) =================
def fig_shadow():
    from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
    sh = []
    for ln in open(os.path.join(HERE, "rtt.log"), errors="replace"):
        m = re.search(r"^(\d+\.\d+)\s.*KSHADOW count=\d+ k_meas=\d+ k0=\d+ dk=(-?\d+)", ln)
        if m:
            sh.append((float(m.group(1)), int(m.group(2))))
    gx, gy = gap_auto_series(os.path.join(HERE, "rtt.log"), os.path.join(HERE, "shadow-dense.shots"))
    t0 = sh[0][0]
    gxm = [(t - t0) / 60 for t in gx]
    g_off = st.mean([g for x, g in zip(gxm, gy) if abs(x) < 1.5])
    gy0 = [g - g_off for g in gy]
    shx = [(t - t0) / 60 for t, _ in sh]
    shy = [-(d - sh[0][1]) * 16 for _, d in sh]

    fig = plt.figure(figsize=(10.2, 8.2))
    gs = fig.add_gridspec(2, 1, height_ratios=[0.8, 1.0], hspace=0.22)

    # ---- 上段: どこのズレを見張るかの模式図 ----
    ax = fig.add_subplot(gs[0]); ax.set_xlim(0, 13); ax.set_ylim(0, 6.4); ax.axis("off")
    C_CPU, C_CPU_E = "#eef2f8", "#446"
    C_PIO, C_PIO_E = "#eaf4ea", "#22661f"
    C_PIN, C_PIN_E = "#f7f7f7", "#666"

    def dbox(x, y, w, h, label, sub=None, ec=C_CPU_E, fc=C_CPU, fs=9.4):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.06", fc=fc, ec=ec, lw=1.3, zorder=4))
        if sub:
            ax.text(x + w / 2, y + h * 0.63, label, ha="center", va="center", fontsize=fs, zorder=5)
            ax.text(x + w / 2, y + h * 0.27, sub, ha="center", va="center", fontsize=7.6, color="#555", zorder=5)
        else:
            ax.text(x + w / 2, y + h / 2, label, ha="center", va="center", fontsize=fs, zorder=5)

    def darrow(p1, p2, color="#446", lw=1.5, ls="-", rad=0.0):
        ax.add_patch(FancyArrowPatch(p1, p2, arrowstyle="->", color=color, lw=lw, ls=ls,
                                     connectionstyle=f"arc3,rad={rad}", zorder=3, shrinkA=2, shrinkB=2))

    # 左から 制御ループ → カウンタ → ピン の順に並べる
    dbox(0.35, 2.45, 2.1, 1.5, "制御ループ", "起動時に測ったズレを\n使い続ける")
    # カウンタ列 (中央)
    dbox(3.2, 4.4, 2.9, 1.2, "GPS 用の捕捉カウンタ", "ずっと GP2 を見る", ec=C_PIO_E, fc=C_PIO)
    dbox(3.2, 0.8, 2.9, 1.2, "loopback の捕捉カウンタ", "ふだんは GP4 を見る", ec=C_PIO_E, fc=C_PIO)
    # ピン列 (右)
    dbox(7.0, 4.4, 1.9, 1.2, "GPS-R の\nPPS ピン (GP2)", None, ec=C_PIN_E, fc=C_PIN, fs=8.6)
    dbox(7.0, 0.8, 1.9, 1.2, "loopback\n入力ピン (GP4)", None, ec=C_PIN_E, fc=C_PIN, fs=8.6)
    darrow((6.95, 5.0), (6.15, 5.0))
    darrow((6.95, 1.4), (6.15, 1.4))
    # 毎秒の読み: カウンタ → 制御ループ
    darrow((3.15, 4.75), (2.15, 3.95), color=C_PIO_E, lw=1.2)
    darrow((3.15, 1.45), (2.15, 2.45), color=C_PIO_E, lw=1.2)
    ax.text(2.2, 4.65, "毎秒の読み", ha="center", fontsize=8.0, color=C_PIO_E)
    ax.text(2.25, 1.35, "毎秒の読み", ha="center", fontsize=8.0, color=C_PIO_E)
    # 60 秒ごとに数秒だけ GP2 を見せる (定期校正と同じ手順)
    darrow((7.5, 4.35), (6.25, 2.0), color="#cc3333", lw=1.5, ls=(0, (4, 2)), rad=-0.25)
    ax.text(9.0, 2.95, "60 秒ごとに数秒だけ、GP2 を\n2 つのカウンタから同時に観測する\n(定期校正と同じ手順)", ha="left",
            fontsize=8.4, color="#cc3333")
    # 読みの差 = 今のズレ (カウンタ 2 本の間で取る)。箱にして「渡さない」の出どころを明確に
    darrow((4.7, 4.32), (4.7, 3.98), color=C_PIO_E, lw=1.4)
    darrow((4.7, 2.08), (4.7, 2.44), color=C_PIO_E, lw=1.4)
    ax.add_patch(FancyBboxPatch((3.45, 2.52), 2.5, 1.38, boxstyle="round,pad=0.06",
                                fc="none", ec=C_PIO_E, lw=1.0, ls=(0, (3, 3)), zorder=4))
    ax.text(4.7, 3.21, "同じエッジの読みの差\n= 今のカウンタのズレ\n→ 記録するだけ", ha="center", va="center",
            fontsize=8.8, color=C_PIO_E, zorder=5)
    ax.text(6.5, 0.15, "カウンタのズレが本当に一定なら、いつ測り直しても起動時と同じ値が出るはず",
            ha="center", fontsize=9.4, color="#333")

    # ---- 下段: 実測 ----
    ax2 = fig.add_subplot(gs[1])
    ax2.plot(gxm, gy0, ".", ms=2, alpha=0.25, color=GREEN)
    ax2.plot(gxm, runmean(gy0, 45), "-", color=DGREEN, lw=1.6,
             label="ピンの上のずれ − 内部位相 (オシロ実測、+5.3 ns/min)")
    ax2.plot(shx, shy, "s-", ms=5.5, color=RED, lw=1.8,
             label="測り直したカウンタのズレ (符号を裏返し、+5.2 ns/min)")
    ax2.set_xlabel("最初の測定からの経過 [min]")
    ax2.set_ylabel("[ns, 0 起点]")
    ax2.grid(ls=":", alpha=0.4); ax2.legend(loc="upper left", fontsize=9)
    fig.savefig(os.path.join(OUT, "fig-shadow-march.png"), dpi=110, bbox_inches="tight"); plt.close(fig)
    print("fig-shadow-march")


# ================= fig-drift-elim =================
def fig_elim():
    # 左: c3gp4 run の c2−c3 (同じ GP4) と gap を同軸で
    kexp = []
    for ln in open(os.path.join(HERE, "c3gp4-rtt.log"), errors="replace"):
        m = KEXP.search(ln)
        if m:
            cnt, c0, c2, c3, c3n = (int(m.group(i)) for i in range(1, 6))
            if c3n == 1 and c0 != 0 and c3 != 0:
                kexp.append((cnt, f32(c2 - c3)))
    med = st.median(d for _, d in kexp)
    kexp = [(c, d) for c, d in kexp if abs(d - med) < 500]
    c00 = kexp[0][0]
    ex = [(c - c00) / 60 for c, _ in kexp]
    ey = [(d - kexp[0][1]) * 16 for _, d in kexp]
    exy = [(x, y) for x, y in zip(ex, ey)]
    gx, gy = gap_auto_series(os.path.join(HERE, "c3gp4-rtt.log"), os.path.join(HERE, "c3gp4-dense.shots"))
    gxm = [(t - gx[0]) / 60 for t in gx]
    # 起動過渡 (ロック整定) を落とし、0 起点は整定後の先頭 1 分で取る
    keep = [i for i in range(len(gxm)) if gxm[i] > 2.0]
    gxm = [gxm[i] for i in keep]; gy = [gy[i] for i in keep]
    gy0 = [g - st.mean(gy[:60]) for g in gy]
    tmax = gxm[-1]
    ex_lim = tmax  # c2−c3 も gap と同じ範囲に切り揃える

    # 右: sh3 run の K03 階段
    kx = {}
    marks = []
    for ln in open(os.path.join(HERE, "sh3-rtt.log"), errors="replace"):
        m = KEXP.search(ln)
        if m:
            kx[int(m.group(1))] = (int(m.group(2)), int(m.group(4)), int(m.group(5)))
            continue
        m = KSH3.search(ln)
        if m:
            marks.append((int(m.group(1)), int(m.group(2))))
    wins = []
    start = None
    for c, on in marks:
        if on == 2:
            start = c
        elif on == 4 and start is not None:
            wins.append((start, c)); start = None
    samples = []
    for s, e in wins:
        ds = []
        for c in range(s + 2, e + 1):
            if c in kx:
                c0, c3, c3n = kx[c]
                if c3n == 1 and c0 != 0 and c3 != 0:
                    ds.append(f32(c0 - c3))
        if len(ds) < 3:
            continue
        m0 = st.median(ds)
        ds = [d for d in ds if abs(d - m0) < 500]
        if len(ds) >= 3:
            samples.append(((s + e) / 2, st.median(ds)))
    k0 = samples[0][1]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.6, 4.4))
    ax1.plot(gxm, gy0, ".", ms=1.5, alpha=0.2, color=GREEN)
    ax1.plot(gxm, runmean(gy0, 45), "-", color=DGREEN, lw=1.6, label="ピンの上のずれ − 内部位相 (オシロ実測、+5.5 ns/min)")
    exy2 = [(x, y) for x, y in exy if 2.0 < x <= ex_lim]
    ax1.plot([x for x, _ in exy2], [y for _, y in exy2], ".", ms=2, color=BROWN, alpha=0.6,
             label="同じ出力ピンを見る 2 本の読み差 (±1 tick)")
    ax1.set_title("ピンの上のずれが増えていく最中も、同じピンを見る 2 本は離れない")
    ax1.set_xlabel("経過 [min]"); ax1.set_ylabel("[ns, 0 起点]")
    ax1.grid(ls=":", alpha=0.4); ax1.legend(loc="upper left", fontsize=8.5)
    xb = 1500 / 60
    A = [sm for sm in samples if sm[0] < 1500]; B = [sm for sm in samples if sm[0] >= 1500]
    slA = slope([c / 60 for c, _ in A], [(k - k0) * 16 for _, k in A])
    slB = slope([c / 60 for c, _ in B], [(k - k0) * 16 for _, k in B])
    ax2.axvline(xb, color="#888", ls="--", lw=1.1)
    ax2.plot([c / 60 for c, _ in samples], [(k - k0) * 16 for _, k in samples],
             "s-", ms=4.5, color=RED, lw=1.3)
    ax2.text(xb / 2, -360, f"60 秒ごとに測る\n{slA:+.1f} ns/min", ha="center", fontsize=9, color="#555")
    ax2.text((xb + samples[-1][0] / 60) / 2, -360, f"240 秒ごとに測る\n{slB:+.1f} ns/min",
             ha="center", fontsize=9, color="#555")
    ax2.set_title("測る間隔を 4 倍にしても、ズレの積もる速さは同じ")
    ax2.set_xlabel("経過 [min]"); ax2.set_ylabel("カウンタのズレの変化 [ns, 0 起点]")
    ax2.set_ylim(-430, 40); ax2.grid(ls=":", alpha=0.4)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig-drift-elim.png"), dpi=110); plt.close(fig)
    print(f"fig-drift-elim: 右 A {slA:+.2f} / B {slB:+.2f} ns/min")


# ================= fig-w900 =================
def fig_w900():
    # 修正前 = 出力 high 100 ms (low 900 ms) の実測 run (同じ firmware、幅だけ違う)
    bx, by = gap_auto_series(os.path.join(LOGS, "20260704-fix-norecal", "rtt.log"),
                             os.path.join(LOGS, "20260704-fix-norecal", "fixnorecal-dense.shots"))
    bxm = [(t - bx[0]) / 60 for t in bx]
    by0 = [g - st.mean(by[:60]) for g in by]
    bi = [i for i in range(len(bxm)) if bxm[i] > 3]
    bsl = slope([bxm[i] for i in bi], [by0[i] for i in bi])
    # 揃えた = 出力 high 900 ms (low 100 ms)
    gx, gy = gap_auto_series(os.path.join(HERE, "w900-rtt.log"), os.path.join(HERE, "w900-dense.shots"))
    gxm = [(t - gx[0]) / 60 for t in gx]
    gy0 = [g - st.mean(gy[:60]) for g in gy]
    gi = [i for i in range(len(gxm)) if gxm[i] > 3]
    gsl = slope([gxm[i] for i in gi], [gy0[i] for i in gi])
    keep = [i for i in range(len(gxm)) if gxm[i] <= 30]
    gxm = [gxm[i] for i in keep]; gy0 = [gy0[i] for i in keep]
    fig, ax = plt.subplots(figsize=(9.8, 4.4))
    ax.plot(bxm, by0, ".", ms=2, alpha=0.2, color="#c05050")
    ax.plot(bxm, runmean(by0, 45), "-", color="#a03030", lw=2,
            label=f"low 900 ms のまま (high 100 ms): {bsl:+.1f} ns/min")
    ax.plot(gxm, gy0, ".", ms=2, alpha=0.25, color=GREEN)
    ax.plot(gxm, runmean(gy0, 45), "-", color=DGREEN, lw=2,
            label=f"low を GPS-R と同じ 100 ms に (high 900 ms): {gsl:+.2f} ns/min")
    # 対称化プログラム版 (幅 100 ms のまま、0 跨ぎを両ループ 3 cycle に)
    wbr = os.path.join(HERE, "wb-rtt.log"); wbs = os.path.join(HERE, "wb-dense.shots")
    if os.path.exists(wbs) and sum(1 for _ in open(wbs)) > 600:
        wx, wy = gap_auto_series(wbr, wbs)
        wxm0 = [(t - wx[0]) / 60 for t in wx]
        # ロック整定 (~2.5 分) 後を t=0 に取り直す (他の線と同じく、整定済みの点から 0 起点で始める)
        kept = [(x - 2.5, g) for x, g in zip(wxm0, wy) if 2.5 <= x <= 32.5]
        wxm = [x for x, _ in kept]
        base = st.mean([g for _, g in kept[:60]])
        wy0 = [g - base for _, g in kept]
        wi = [i for i in range(len(wxm)) if wxm[i] > 0.5]
        wsl = slope([wxm[i] for i in wi], [wy0[i] for i in wi])
        ax.plot(wxm, wy0, ".", ms=2, alpha=0.22, color="#5b7fc7")
        ax.plot(wxm, runmean(wy0, 45), "-", color="#2a5db0", lw=2,
                label=f"0 跨ぎを両ループ 3 cycle に (幅は 100 ms のまま): {wsl:+.2f} ns/min")
        print(f"fig-w900: wb {wsl:+.2f} ns/min")
    ax.set_xlabel("経過 [min]")
    ax.set_ylabel("ピンの上のずれ − 内部位相 [ns, 0 起点]")
    ax.set_title("一周で遅れる量を 2 本のカウンタで揃えると、ずれは止まる")
    ax.grid(ls=":", alpha=0.4); ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig-w900.png"), dpi=110); plt.close(fig)
    print(f"fig-w900: 100ms {bsl:+.2f} / 900ms {gsl:+.2f} ns/min")


fig_inject()
fig_shadow()
fig_elim()
fig_w900()
