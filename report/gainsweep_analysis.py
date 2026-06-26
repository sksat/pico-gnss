#!/usr/bin/env python3
"""制御器+ゲイン掃引 (interleave + PRBS) のログを config 別に解析する。

各 config について次を出す:
  - PIO steady σ          firmware hwphase (output-GPS, PIO 16ns 捕捉) の定常ばらつき
  - scope σ               オシロが独立に測った同じ output-GPS の σ (第3計器の裏取り)
  - 共振ゲイン/@周期/coh   PRBS 注入と hwphase の Welch 閉ループ周波数応答 (受信非依存)
  - jit / 温度             受信条件と基板温度 (config 間で交絡してないかの点検)

scope σ の対応づけは firmware の RTT 時計と scope の wall-time を **相互相関**で較正してから行う:
両者は同じ output-GPS を同じパルスで測っているので、相関ピークのラグが真の時計差、ピーク相関値が
「同じパルスを見ている」証拠になる (ホスト NTP 同期前提。scope_logger.py 参照)。

env: FW_LOG (既定 logs/pps-gainsweep.log), SCOPE_LOG (既定 logs/scope-gainsweep.log)。
CFG は掃引した make_controller の idx→説明。掃引内容を変えたらここを合わせる。
"""
import os, re
import numpy as np

FW = os.environ.get("FW_LOG", "logs/pps-gainsweep.log")
SC = os.environ.get("SCOPE_LOG", "logs/scope-gainsweep.log")
CFG = {0: "iden128/d4", 1: "iden512/d16(prod)", 2: "iden1024/d64",
       3: "iden512/d4", 4: "ab_boost", 5: "integ_rework"}

def t2c(raw):  # RP2040 内蔵温度センサ raw(12bit ADC)→ ℃ (データシート式)
    v = raw * 3.3 / 4096
    return 27 - (v - 0.706) / 0.001721 if raw > 0 else float("nan")

# --- firmware ログ: (rtt, hwphase, inj, cidx, locked, jit, temp_raw) ---
rows = []
for line in open(FW, errors="ignore"):
    if "PPSGEN" not in line:
        continue
    rt = re.match(r"\s*([\d.]+)\s", line)
    def g(k, d=0):
        m = re.search(rf"\b{k}=(-?\d+)\b", line); return int(m.group(1)) if m else d
    hw = re.search(r"hwphase_ns=(-?\d+)", line); inj = re.search(r"inj_ns=(-?\d+)", line)
    cx = re.search(r"\bcidx=(\d+)\b", line); lk = re.search(r"lk=(\d)", line)
    if rt and hw and inj and cx and lk:
        rows.append((float(rt.group(1)), int(hw.group(1)), int(inj.group(1)),
                     int(cx.group(1)), int(lk.group(1)), g("jit"), g("temp_raw")))

# 連続 cidx でセグメント化し RTT 範囲を保持: (cidx, lo, hi, rtt_start, rtt_end)
segs = []; i = 0
while i < len(rows):
    j = i
    while j < len(rows) and rows[j][3] == rows[i][3]:
        j += 1
    if j - i >= 200:
        segs.append((rows[i][3], i, j, rows[i][0], rows[j - 1][0]))
    i = j
by_cfg = {}
for s in segs:
    by_cfg.setdefault(s[0], []).append(s)

# --- scope ログ + RTT 橋 ---
scope = []; sc_off = None
try:
    for line in open(SC, errors="ignore"):
        if line.startswith("#"):
            m = re.search(r"RTT = wall - ([\d.]+)", line)
            if m: sc_off = float(m.group(1))
            continue
        p = line.split()
        if len(p) == 2 and sc_off is not None:
            scope.append((float(p[0]) - sc_off, float(p[1])))  # (firmware-RTT 換算, offset)
except FileNotFoundError:
    pass
scope = np.array(scope) if scope else np.empty((0, 2))

# --- 相互相関でラグ自動較正: firmware hwphase vs scope offset ---
lag = 0; xr = float("nan")
if len(scope) > 50:
    fw = np.array([(r[0], r[1]) for r in rows if r[4] == 1 and abs(r[1]) < 5000])
    sc = scope[np.argsort(scope[:, 0])]
    best = (0, -9, 0)
    for d in range(-25, 26):
        pairs = []
        for t, h in fw:
            ts = t - d
            k = np.searchsorted(sc[:, 0], ts)
            cand = [x for x in (k - 1, k) if 0 <= x < len(sc)]
            if not cand: continue
            kk = min(cand, key=lambda x: abs(sc[x, 0] - ts))
            if abs(sc[kk, 0] - ts) < 0.75:
                pairs.append((h, sc[kk, 1]))
        if len(pairs) >= 30:
            a = np.array(pairs)
            if a[:, 0].std() > 1 and a[:, 1].std() > 1:
                r = np.corrcoef(a[:, 0], a[:, 1])[0, 1]
                if r > best[1]: best = (d, r, len(pairs))
    lag, xr, _ = best
    scope[:, 0] -= lag  # scope の RTT をラグ補正して firmware に厳密整合

def welch(slist, nfft=256):
    cr = np.zeros(nfft // 2 + 1, complex); ai = np.zeros(nfft // 2 + 1); ah = np.zeros(nfft // 2 + 1)
    cnt = 0; win = np.hanning(nfft)
    for _, lo, hi, _, _ in slist:
        hw = np.array([rows[k][1] for k in range(lo + 60, hi) if rows[k][4] == 1], float)
        ij = np.array([rows[k][2] for k in range(lo + 60, hi) if rows[k][4] == 1], float)
        if len(hw) < nfft: continue
        hw -= hw.mean(); ij -= ij.mean()
        for st in range(0, len(hw) - nfft, nfft // 2):
            H = np.fft.rfft(hw[st:st + nfft] * win); I = np.fft.rfft(ij[st:st + nfft] * win)
            cr += H * np.conj(I); ai += np.abs(I) ** 2; ah += np.abs(H) ** 2; cnt += 1
    if cnt < 2: return None
    f = np.fft.rfftfreq(nfft, 1.0)
    return f, np.abs(cr) / np.maximum(ai, 1e-9), np.abs(cr) ** 2 / np.maximum(ai * ah, 1e-12)

# --- 温度サマリ (内蔵センサは単発 ±数℃ とノイジー: min/max でなく中央/IQR と平滑後で見る) ---
temps = [t2c(r[6]) for r in rows if r[6] > 0]
if temps:
    ta = np.array(temps); q1, med, q3 = np.percentile(ta, [25, 50, 75])
    sm = np.convolve(ta, np.ones(60) / 60, "valid") if len(ta) > 60 else ta
    print(f"# FW {FW}: 温度 中央{med:.1f}℃ IQR[{q1:.1f},{q3:.1f}] | 平滑後の真スイング "
          f"{sm.min():.1f}-{sm.max():.1f}℃ (生 min/max {ta.min():.1f}-{ta.max():.1f} は単発 ADC ノイズ)")
else:
    print("# 温度未取得")
print(f"# scope {SC}: {len(scope)} shots | 相互相関ラグ={lag}s r={xr:+.2f} "
      f"(r 高=同じパルスを測っている=対応づけ確定, ラグ補正適用済)")
print("# 各 config セグメント数: " + " ".join(f"{c}:{len(by_cfg.get(c, []))}" for c in sorted(CFG)))
print(f"\n{'config':>18} {'seg':>3} {'PIO σ':>6} {'scope σ':>7} {'共振G':>6} {'@周期':>6} {'coh':>5} {'jit':>4} {'温度':>7}")
for c in sorted(by_cfg):
    sl = by_cfg[c]
    hw = [rows[k][1] for _, lo, hi, _, _ in sl for k in range(lo + 60, hi)
          if rows[k][4] == 1 and abs(rows[k][1]) < 5000]
    jit = [rows[k][5] for _, lo, hi, _, _ in sl for k in range(lo, hi)]
    tp = [t2c(rows[k][6]) for _, lo, hi, _, _ in sl for k in range(lo, hi) if rows[k][6] > 0]
    sc_sigs = []
    for _, lo, hi, t0, t1 in sl:
        v = scope[(scope[:, 0] >= t0) & (scope[:, 0] <= t1), 1] if len(scope) else []
        if len(v) >= 10: sc_sigs.append(np.std(v))  # セグメント内 raw σ=wander (固定 skew は区間内一定)
    scs = np.median(sc_sigs) if sc_sigs else float("nan")
    w = welch(sl); gn = pp = co = float("nan")
    if w:
        f, Hf, coh = w; band = (f > 1 / 256) & (f < 1 / 30); bi = np.where(band)[0]
        pk = bi[np.argmax(Hf[bi])]; gn = Hf[pk]; pp = 1 / f[pk]; co = coh[pk]
    sd = np.std(hw) if len(hw) > 50 else float("nan")
    print(f"{CFG[c]:>18} {len(sl):>3} {sd:>6.0f} {scs:>7.0f} {gn:>6.1f} {pp:>5.0f}s {co:>5.2f} "
          f"{int(np.median(jit)) if jit else -1:>4} {np.median(tp) if tp else float('nan'):>6.1f}℃")

# 温度 vs hwphase σ ブロック相関 (wander に熱成分があるか。ブロック平均で ADC ノイズを均す)
B = 300
H = np.array([r[1] for r in rows if r[4] == 1 and abs(r[1]) < 5000 and r[6] > 0])
T = np.array([t2c(r[6]) for r in rows if r[4] == 1 and abs(r[1]) < 5000 and r[6] > 0])
if len(H) > 2000:
    nb = len(H) // B
    bs = np.array([H[i * B:(i + 1) * B].std() for i in range(nb)])
    bt = np.array([T[i * B:(i + 1) * B].mean() for i in range(nb)])
    rr = np.corrcoef(bt, bs)[0, 1] if bt.std() > 1e-3 else float("nan")
    print(f"\n# 温度 vs hwphase σ ブロック相関 r={rr:+.2f} ({nb}ブロック) → |r|大なら wander に熱成分")
print("# PIO σ と scope σ が config 間で同順なら、PIO 相対測定が独立計器で裏取りされた。")
print("# iden1024/d64(最 overdamp)が prod より PIO σ・共振 G とも明確低 →『ゲインで削れる』。"
      "全 config 誤差内 →『受信律速』。")
