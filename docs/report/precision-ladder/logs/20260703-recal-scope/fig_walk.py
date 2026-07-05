#!/usr/bin/env python3
"""recal なし/ありの gap (実効 K の誤差 = オシロ実測 − 内部 loopback 位相) の最終図。
左: stage-3 (起動時校正のみ) の 20 分ごとのチェックポイント。ずれていく。t100 は計測不良として灰色。
右: stage-5 (recal あり) の 40 分連続計測。緑線 = RTT から抽出した recal 全 17 回。平坦。
usage: uv run --with matplotlib python3 logs/20260703-recal-scope/fig_walk.py"""
import os, re, time, statistics as st
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",):
    if os.path.exists(fp):
        try: font_manager.fontManager.addfont(fp)
        except Exception: pass
plt.rcParams["font.family"] = "Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"] = False
HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = os.path.dirname(os.path.dirname(HERE))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(REPORT)))
DATA = os.path.join(ROOT, "logs", "20260703-recal-scope")  # 生データ (gitignore、ローカルのみ)
OUT = os.path.join(REPORT, "precision-figs")

def wall(hms):
    return time.mktime(time.strptime(f"2026-07-03 {hms}", "%Y-%m-%d %H:%M:%S"))

def load_rtt(path):
    out = []
    for ln in open(path, errors="replace"):
        if "PPSGEN count=" not in ln:
            continue
        m = re.match(r"^(\d+\.\d+)\s", ln)
        if not m:
            continue
        d = {k: int(v) for k, v in re.findall(r"(\w+)=(-?\d+)", ln)}
        out.append((float(m.group(1)), d["hwphase_ns"]))
    return out

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.5, 4.4), sharey=True)

# ---------- 左: stage-3 (起動時校正のみ) ----------
BOOT3 = wall("09:17:10")
rtt3 = load_rtt(os.path.join(DATA, "stage3-rtt.log"))
pts = []
for tag, bad in [("s3-t000", 0), ("s3-t020", 0), ("s3-t040", 0), ("s3-t060", 0), ("s3-t080", 0), ("s3-t100", 1)]:
    shots = [(float(a), float(b)) for a, b in (ln.split() for ln in open(os.path.join(DATA, f"{tag}.shots")))]
    good = [v for _, v in shots if abs(v) < 450]
    w0, w1 = min(t for t, _ in shots), max(t for t, _ in shots)
    inner = [h for ts, h in rtt3 if w0 <= BOOT3 + ts <= w1]
    gap = st.median(good) - st.mean(inner)
    pts.append(((w0 + w1) / 2, gap, bad))
t0 = pts[0][0]
xs = [(t - t0) / 60 for t, _, _ in pts]
ys = [g for _, g, _ in pts]
ok = [(x, y) for (x, y), (_, _, b) in zip(zip(xs, ys), pts) if not b]
ng = [(x, y) for (x, y), (_, _, b) in zip(zip(xs, ys), pts) if b]
ax1.plot([x for x, _ in ok], [y for _, y in ok], "o-", color="#c05050", ms=8, lw=1.5)
for x, y in ng:
    ax1.plot(x, y, "o", color="#bbb", ms=8)
    ax1.annotate("計測不良\n(shot の半数が失敗)", xy=(x, y), xytext=(x - 28, y - 60), fontsize=9, color="#888",
                 arrowprops=dict(arrowstyle="->", color="#aaa"))
n = len(ok); mx = sum(x for x, _ in ok) / n; my = sum(y for _, y in ok) / n
sl = sum((x - mx) * (y - my) for x, y in ok) / sum((x - mx) ** 2 for x, _ in ok)
ax1.plot([ok[0][0], ok[-1][0]], [my + sl * (ok[0][0] - mx), my + sl * (ok[-1][0] - mx)], "--", color="#c05050", lw=1.2)
ax1.text(35, 300, f"約 {sl:+.1f} ns/min でずれていく", color="#a03030", fontsize=11)
ax1.set_title("起動時校正のみ (20 分ごとに 30 発の中央値)")
ax1.set_xlabel("最初のチェックポイントからの経過 [分]")
ax1.set_ylabel("ピンの上のずれ − 内部の loopback 位相  [ns]")
ax1.grid(ls=":", alpha=0.4)

# ---------- 右: stage-5 (recal あり、連続) ----------
BOOT5 = wall("10:57:30")
rtt5 = load_rtt(os.path.join(DATA, "stage5-rtt.log"))
rtt5_w = [(BOOT5 + ts, h) for ts, h in rtt5]
shots5 = [(float(a), float(b)) for a, b in (ln.split() for ln in open(os.path.join(DATA, "s5-dense.shots")))]
t0d = shots5[0][0]
gx, gy = [], []
j = 0
for t, v in shots5:
    while j + 1 < len(rtt5_w) and abs(rtt5_w[j + 1][0] - t) <= abs(rtt5_w[j][0] - t):
        j += 1
    if abs(rtt5_w[j][0] - t) <= 2.5:
        gx.append((t - t0d) / 60); gy.append(v - rtt5_w[j][1])
recals = [BOOT5 + float(m.group(1)) for m in
          (re.match(r"^(\d+\.\d+)\s", ln) for ln in open(os.path.join(DATA, "stage5-rtt.log"), errors="replace")
           if "PHASE_K recal OK" in ln) if m]
first = True
for rt in recals:
    x = (rt - t0d) / 60
    if 0 <= x <= gx[-1]:
        ax2.axvline(x, color="#2a9d2a", ls=":", lw=1.2, label="定期校正" if first else None)
        first = False
ax2.plot(gx, gy, ".", color="#4a6fb0", ms=2.5, alpha=0.6)
n = len(gx); mx = sum(gx) / n; my = sum(gy) / n
sl2 = sum((x - mx) * (y - my) for x, y in zip(gx, gy)) / sum((x - mx) ** 2 for x in gx)
ax2.plot([gx[0], gx[-1]], [my + sl2 * (gx[0] - mx), my + sl2 * (gx[-1] - mx)], "--", color="#333", lw=1.4)
ax2.text(12, 300, f"約 {sl2:+.1f} ns/min にとどまる", color="#333", fontsize=11)
ax2.set_title("定期校正あり (連続計測。緑線 = 校正の実行、全 %d 回)" % sum(1 for rt in recals if 0 <= (rt - t0d) / 60 <= gx[-1]))
ax2.set_xlabel("計測開始からの経過 [分]")
ax2.legend(loc="lower right", fontsize=9)
ax2.grid(ls=":", alpha=0.4)
ax1.set_ylim(-150, 480)

fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig-recal-walk.png"), dpi=110)
print(f"left slope={sl:+.2f} ns/min  right slope={sl2:+.2f} ns/min  right n={n} recal lines={sum(1 for rt in recals if 0 <= (rt-t0d)/60 <= gx[-1])}")
