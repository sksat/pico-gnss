#!/usr/bin/env python3
"""PRBS 自己同定の解析(scipy 不要)。

ロック中に出力位相へ注入した既知の擬似ランダム外乱 inj_ns と、その応答 hwphase の相互相関を取ると、
受信ノイズ(inj と非相関 → 平均で落ちる)を分離して **閉ループ外乱インパルス応答 h[k]** が出る。
h[k] のリンギング(負へオーバーシュート→振動)= ループの実効 under-damping=潰せる共振。config(i_den)別に
比べ、firmware loop にまだ制御可能な共振が残っているかを受信交絡なしに判定する。座標(NMEA)は読み捨て。

  python3 report/analyze_prbs.py logs/pps-prbs.log
"""
import re, sys, statistics

LOG = sys.argv[1] if len(sys.argv) > 1 else "logs/pps-prbs.log"
MAXLAG = 60
WARMUP = 120  # 切替後に捨てる整定エッジ(手法に共通の固定 warmup ≳2 周期。non-PLL は固有周期が無いため固定)

# 制御器インデックス → 名前(firmware の make_controller と対応。表示用)。
CIDX_NAMES = {0: "pll_prod", 1: "ab_boost", 2: "integ_rework", 3: "pll_smith128", 4: "naive_pid"}

rows = []  # (hw, inj, cidx, lk)
for l in open(LOG, errors="ignore"):
    if "PPSGEN" not in l:
        continue
    hw = re.search(r"\bhwphase_ns=(-?\d+)\b", l)
    inj = re.search(r"\binj_ns=(-?\d+)\b", l)
    cidx = re.search(r"\bcidx=(\d+)\b", l)
    lk = re.search(r"\blk=(-?\d+)\b", l)
    if hw and inj and cidx and lk:
        rows.append((int(hw.group(1)), int(inj.group(1)), int(cidx.group(1)), int(lk.group(1))))
if not rows:
    sys.exit("no PPSGEN inj_ns/cidx rows (need PRBS + CONTROLLER_SWEEP firmware)")


def name(cidx):
    return CIDX_NAMES.get(cidx, f"c{cidx}")


# cidx でセグメント分割、各制御器の locked サンプルを束ねる(切替直後の warmup を捨てる)
segs = []
i = 0
while i < len(rows):
    j = i
    while j < len(rows) and rows[j][2] == rows[i][2]:
        j += 1
    segs.append((rows[i][2], i, j))
    i = j
print(f"# {LOG}: {len(rows)} PPSGEN rows, segments: " +
      " ".join(f"{name(v)}({h-l}e)" for v, l, h in segs))

by_iden = {}  # 旧名のまま(下流の表ロジックは制御器キーで再利用)
for cidx, lo, hi in segs:
    seg = [(rows[k][0], rows[k][1]) for k in range(lo + WARMUP, hi) if rows[k][3] == 1]
    if len(seg) > 100:
        by_iden.setdefault(cidx, []).append(seg)


def xcorr_impulse(pairs, maxlag):
    """inj→hwphase の閉ループ外乱インパルス応答 h[k]=E[inj[n]*hw[n+k]]/E[inj^2]。連結境界を跨がない。"""
    num = [0.0] * maxlag
    den = 0.0
    for seg in pairs:
        hw = [p[0] for p in seg]
        inj = [p[1] for p in seg]
        mi = statistics.mean(inj)
        mh = statistics.mean(hw)
        inj = [v - mi for v in inj]
        hw = [v - mh for v in hw]
        den += sum(v * v for v in inj)
        n = len(seg)
        for k in range(maxlag):
            num[k] += sum(inj[t] * hw[t + k] for t in range(n - k))
    return [x / den for x in num] if den else None


print(f"\n{'controller':>13} {'Nseg':>5} {'Nsamp':>7} {'peak h':>7} {'@lag':>5} {'min h':>7} {'@lag':>5} "
      f"{'ring(zc)':>8} {'振動?':>6}")
results = {}
for cidx in sorted(by_iden):
    pairs = by_iden[cidx]
    nsamp = sum(len(s) for s in pairs)
    h = xcorr_impulse(pairs, MAXLAG)
    if not h:
        continue
    results[cidx] = h
    pk = max(h); pkl = h.index(pk)
    mn = min(h); mnl = h.index(mn)
    # リンギング: h[k] のゼロ交差数(多い=振動=under-damped)。ピーク以後で数える。
    zc = sum(1 for k in range(pkl, len(h) - 1) if (h[k] > 0) != (h[k + 1] > 0))
    osc = "あり" if (mn < -0.08 and mnl > pkl) else "なし"  # ピーク後に明確な負のアンダーシュート=共振
    print(f"{name(cidx):>13} {len(pairs):>5} {nsamp:>7} {pk:>7.2f} {pkl:>5} {mn:>7.2f} {mnl:>5} "
          f"{zc:>8} {osc:>6}")

# h[k] の先頭を並べて見る(リンギングの直接確認)
print("\n# 閉ループ外乱インパルス応答 h[k](k=0..15, ×100):")
for cidx in sorted(results):
    h = results[cidx]
    print(f"  {name(cidx):>13}: " + " ".join(f"{v*100:+5.0f}" for v in h[:16]))
print("# 解釈: h はピーク(k≈1)後に単調減衰なら well-damped。負へアンダーシュート→振動なら共振=潰せる余地あり。")
