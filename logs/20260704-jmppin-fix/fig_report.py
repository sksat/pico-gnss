#!/usr/bin/env python3
"""レポート用図 2 枚: 同値書き込み実験の階段と、切替修正の前後比較。
usage: uv run --with matplotlib python3 logs/20260704-jmppin-fix/fig_report.py
"""
import os
import re
import statistics as st

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

for _fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/noto-cjk/NotoSansCJK-Light.ttc"):
    if os.path.exists(_fp):
        font_manager.fontManager.addfont(_fp)
plt.rcParams["font.family"] = "Noto Sans CJK JP"

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.dirname(HERE)
FIGS = os.path.join(LOGS, "..", "docs", "report", "precision-ladder", "precision-figs")

KEXP = re.compile(r"KEXP count=(\d+) gen=\d+ c0=(\d+) c2=(\d+) c3=(\d+) c3n=(\d+)")
KPOKE = re.compile(r"KPOKE kind=(\w+) n=(\d+)")


def to_i32(u):
    u &= 0xFFFFFFFF
    return u - (1 << 32) if u >= (1 << 31) else u


# ---- 図 1: 同値書き込み実験 (KPOKE) の K_same 階段 ----
events = []
for line in open(os.path.join(LOGS, "20260704-kpoke", "rtt.log"), errors="replace"):
    m = KEXP.search(line)
    if m:
        c0, c3, c3n = int(m.group(2)), int(m.group(4)), int(m.group(5))
        if c3n >= 1:
            events.append(("k", to_i32(c0 - c3)))
        continue
    m = KPOKE.search(line)
    if m:
        events.append(("p", m.group(1), int(m.group(2))))

ks, pokes = [], []  # pokes: (ks_index, kind, n)
for e in events:
    if e[0] == "k":
        ks.append(e[1])
    else:
        pokes.append((len(ks), e[1], e[2]))
# full poke 直後の 1 サンプルは偽キャプチャ値 (FIFO 満杯で真エッジの push が落ちた回) が
# 混ざり ±1e9 ns 級に飛ぶので、中央値から大きく外れる点を落とす
med = st.median(ks)
keep = [abs(k - med) < 500 for k in ks]  # 真の階段の全幅は ~70 tick
ks = [k for k, f in zip(ks, keep) if f]
shift = [sum(1 for f in keep[:i] if not f) for i in range(len(keep))]
pokes = [(pos - shift[min(pos, len(shift) - 1)], kind, n) for pos, kind, n in pokes]
k0 = ks[0]
ks_ns = [(k - k0) * 16 for k in ks]
t_min = [i / 60 for i in range(len(ks))]

fig, ax = plt.subplots(figsize=(9.5, 4.2))
ax.plot(t_min, ks_ns, lw=0.9, color="#1f77b4",
        label="同じ GP2 を見る 2 本 (c0 と c3) の読みの差")
seen = set()
for pos, kind, n in pokes:
    if kind == "full":
        color = "#d62728" if n == 4 else "#ff9896"
        lbl = f"観測ピン切替関数を {n} 回呼ぶ"
    else:
        color = "#bbbbbb"
        lbl = "レジスタを 1 本だけ書く (同値、4 回)"
    ax.axvline(pos / 60, color=color, lw=1.1, alpha=0.85,
               label=None if lbl in seen else lbl, zorder=0)
    seen.add(lbl)
p_full4 = pokes[19]  # 4 巡目の観測ピン切替関数 4 回
y4 = ks_ns[min(p_full4[0] + 8, len(ks_ns) - 1)]
ax.annotate("観測ピン切替関数を呼ぶたびに読みが下がる\n(カウンタが遅れる。4 回で −128 ns = −8 tick)",
            xy=(p_full4[0] / 60, y4), xytext=(21, y4 + 210),
            fontsize=9, color="#d62728", arrowprops=dict(arrowstyle="->", color="#d62728", lw=0.9))
p_reg = pokes[9]  # レジスタ単体 (灰線)
yr = ks_ns[min(p_reg[0] + 8, len(ks_ns) - 1)]
ax.annotate("レジスタを 1 本だけ書いても下がらない",
            xy=(p_reg[0] / 60, yr), xytext=(2.5, yr - 320),
            fontsize=9, arrowprops=dict(arrowstyle="->", lw=0.8))
ax.set_xlabel("経過 [min]")
ax.set_ylabel("2 本の読みの差の変化 [ns]  (tick = 16 ns)")
ax.set_title("カウンタが遅れる原因の切り分け")
ax.legend(fontsize=8, loc="lower left")
ax.grid(alpha=0.25)
fig.tight_layout()
fig.savefig(os.path.join(FIGS, "fig-kpoke-poke.png"), dpi=110)
print("fig-kpoke-poke.png")

# ---- 図 2: 切替修正の前後 (dk 分布とピンの上のズレ) ----
def dks(path):
    out = []
    for line in open(path, errors="replace"):
        if "recal OK" in line:
            m = re.search(r"dk=(-?\d+)", line)
            if m:
                out.append(int(m.group(1)))
    return out


def shots(path, skip_s=300):
    rows = [line.split() for line in open(path)]
    t0 = float(rows[0][0])
    return [float(v) for t, v in rows if float(t) > t0 + skip_s]


dk_before = dks(os.path.join(LOGS, "20260703-kexp", "kexp-run.log"))
dk_after = dks(os.path.join(LOGS, "20260704-jmppin-fix", "rtt.log"))
g_before = shots(os.path.join(LOGS, "20260704-kpoke", "kpoke-dense.shots"))
g_after = shots(os.path.join(LOGS, "20260704-jmppin-fix", "fix-dense.shots"))

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4.0))
vals = sorted(set(dk_before) | set(dk_after))
w = 0.38
ax1.bar([v - w / 2 for v in vals], [dk_before.count(v) / len(dk_before) for v in vals],
        width=w, color="#888888", label=f"修正前 ({len(dk_before)} 回)")
ax1.bar([v + w / 2 for v in vals], [dk_after.count(v) / len(dk_after) for v in vals],
        width=w, color="#2ca02c", label=f"修正後 ({len(dk_after)} 回)")
ax1.set_xlabel("1 回の定期校正で測れたカウンタのズレ [目盛り (16 ns)]")
ax1.set_ylabel("割合")
ax1.set_title("定期校正 1 回あたりのカウンタのズレの測り直し量")
ax1.set_ylim(0, 1.02)
ax1.annotate("ぴったり −4 が 42/56", xy=(-4.2, 0.75), xytext=(-6.2, 0.58),
             fontsize=9, arrowprops=dict(arrowstyle="->", lw=0.8))
ax1.annotate("−1 (実ドリフトのみ) が 20/23", xy=(-0.8, 0.87), xytext=(-4.6, 0.94),
             fontsize=9, arrowprops=dict(arrowstyle="->", lw=0.8))
ax1.legend(fontsize=8, loc="upper left")
ax1.grid(alpha=0.25, axis="y")

mb, ma = st.mean(g_before), st.mean(g_after)
bins = [x for x in range(-250, 351, 12)]
ax2.hist(g_before, bins=bins, density=True, alpha=0.55, color="#888888",
         label=f"修正前 (中心 {mb:+.0f} ns、{len(g_before)} 発)")
ax2.hist(g_after, bins=bins, density=True, alpha=0.55, color="#2ca02c",
         label=f"修正後 (中心 {ma:+.0f} ns、{len(g_after)} 発)")
ax2.axvline(mb, color="#555555", lw=1.2, ls="--")
ax2.axvline(ma, color="#1a7a1a", lw=1.2, ls="--")
ax2.annotate("", xy=(ma, 0.0062), xytext=(mb, 0.0062),
             arrowprops=dict(arrowstyle="->", lw=1.2))
ax2.text((mb + ma) / 2, 0.0064, f"{ma - mb:+.0f} ns", ha="center", fontsize=9)
ax2.set_xlabel("出力と GPS-R のピンの上のズレ (オシロ実測) [ns]")
ax2.set_ylabel("密度")
ax2.set_title("ピンの上のズレ (収束後 40〜55 分)")
ax2.legend(fontsize=8)
ax2.grid(alpha=0.25, axis="y")
fig.tight_layout()
fig.savefig(os.path.join(FIGS, "fig-kslip-fix.png"), dpi=110)
print("fig-kslip-fix.png")
