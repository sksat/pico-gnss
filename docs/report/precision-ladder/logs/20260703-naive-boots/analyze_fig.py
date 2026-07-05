#!/usr/bin/env python3
"""タイマーだけの 1PPS (stage 0) を 8 回起動した実験の図。
PPSGEN の hwphase_ns は stage 0 ではソフト計測 (GPIO 割込 + Instant の秒内 fold) — PIO は動いていない。
左: 起動直後の位相差 (ms) を起動ごとに並べる -> バラバラ (オフセットのズレ)。
右: 各起動の位相差の変化 (µs) を重ねる -> 進む速さは毎回ほぼ同じ ~ -3.2 µs/s (間隔のズレ)。
usage: uv run --with matplotlib python3 logs/20260703-naive-boots/analyze_fig.py
"""
import re, os, glob, statistics as st
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
DATA = os.path.join(ROOT, "logs", "20260703-naive-boots")  # 生データ (gitignore、ローカルのみ)
OUT = os.path.join(REPORT, "precision-figs")

def load(path, warm=5):
    xs, hw, dev = [], [], []
    for ln in open(path, errors="replace"):
        if "PPSGEN count=" not in ln:
            continue
        d = {k: int(v) for k, v in re.findall(r"(\w+)=(-?\d+)", ln)}
        if d.get("count", 0) > warm:
            xs.append(d["count"]); hw.append(d["hwphase_ns"]); dev.append(d["dev_ns"])
    if not xs:
        return None
    x0 = xs[0]
    # fold(±500ms) の折り返しをほどく (境界近くで start した boot 用)
    for i in range(1, len(hw)):
        while hw[i] - hw[i-1] > 500_000_000: hw[i] -= 1_000_000_000
        while hw[i] - hw[i-1] < -500_000_000: hw[i] += 1_000_000_000
    return [x - x0 for x in xs], hw, dev

boots = []
for p in sorted(glob.glob(os.path.join(DATA, "boot*.log"))):
    r = load(p)
    if r:
        boots.append((os.path.basename(p).replace(".log", ""), *r))

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4.2))
cmap = plt.get_cmap("tab10")
slopes = []

# 左: 起動直後の位相差 (バラバラ)
for i, (name, xs, hw, dev) in enumerate(boots):
    ax1.scatter([i + 1], [hw[0] / 1e6], color=cmap(i), s=70, zorder=3)
ax1.axhline(0, color="#999", lw=0.8, ls=":")
ax1.set_xlabel("何回目の起動か"); ax1.set_ylabel("起動直後の位相差 (ms)")
ax1.set_xticks(range(1, len(boots) + 1))
ax1.set_ylim(-520, 520); ax1.set_yticks([-500, -250, 0, 250, 500])
ax1.set_title("最初のずれは起動ごとにバラバラ")
ax1.grid(axis="y", ls=":", alpha=0.4)

# 右: 位相差の変化 (毎回同じ速さで進む)
for i, (name, xs, hw, dev) in enumerate(boots):
    dy = [(v - hw[0]) / 1e3 for v in hw]
    n = len(xs); mx = sum(xs) / n; my = sum(dy) / n
    sl = sum((a - mx) * (b - my) for a, b in zip(xs, dy)) / sum((a - mx) ** 2 for a in xs)
    slopes.append(sl * 1e3)
    ax2.plot(xs, dy, color=cmap(i), lw=1.4, alpha=0.85)
    print(f"{name}: init={hw[0]/1e6:+8.1f} ms  slope={sl*1e3:+7.1f} ns/s  n={n}  "
          f"dev mean={st.mean(dev):+7.1f} sd={st.pstdev(dev):7.1f}")
ax2.set_xlabel("経過時間 (s)"); ax2.set_ylabel("起動直後からの位相差の変化 (µs)")
ax2.set_title(f"位相差が動く速さは毎回ほぼ同じ (平均 {st.mean(slopes)/1e3:+.1f} µs/s)")
ax2.grid(ls=":", alpha=0.4)

fig.suptitle(f"タイマーだけの 1PPS と GPS-R 1PPS のずれ ({len(boots)} 回起動、ソフト計測)", fontsize=12)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig-naive-phase.png"), dpi=110)
print(f"slopes: mean={st.mean(slopes):+.1f} ns/s sd={st.pstdev(slopes):.1f} ns/s")
print("wrote fig-naive-phase.png")
