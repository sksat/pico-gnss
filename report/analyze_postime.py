#!/usr/bin/env python3
"""position-time coupling の実機検証(座標を一切出さない)。

仮説: 出力位相の遅い wander は、受信機が毎秒 (x,y,z,t) を同時に解くために**位置誤差が時刻解へ漏れる**
(position-time coupling)もの。これが本当なら、NMEA の**位置偏差**(平均からのズレ)と、ループの制御誤差
hwphase / ドリフト推定 slope / 操舵 trim の遅い成分が相関するはず。とくに鉛直(up)は衛星幾何で時刻と
最も強く結合する。相関すれば → 「静止拘束(survey-in/position-hold)で位置を固定すれば時刻 wander を断てる」
が定量的に裏づく。

**座標保護**: 緯度経度高度の絶対値は一切出力しない。出すのは平均からの偏差(meter)・相関係数・遅延構造だけ。
NMEA(GGA)と PPSGEN を行頭の RTT タイムスタンプで対応づける。

  python3 report/analyze_postime.py logs/pps-prod-clean.log
"""
import re, sys, math
import numpy as np

LOG = sys.argv[1] if len(sys.argv) > 1 else "logs/pps-prod-clean.log"
SLOW_WIN = 30  # 遅い成分を見る移動平均窓(エッジ≈秒)
MAXLAG = 20    # ラグ相関の範囲(秒)

# --- パース: GGA(位置)と PPSGEN(制御量)を RTT タイムスタンプ付きで ---
gga = []  # (t, lat_deg, lon_deg, alt_m)
pps = []  # (t, hwphase, lk, slope, trim, dev)
ts_re = re.compile(r"^\s*([\d.]+)\s")
for line in open(LOG, errors="ignore"):
    m = ts_re.match(line)
    if not m:
        continue
    t = float(m.group(1))
    if "GGA" in line:
        # $..GGA,time,llll.llll,N,yyyyy.yyyy,E,fix,nsat,hdop,alt,M,...
        f = line.split("GGA,", 1)[1].split(",") if "GGA," in line else None
        if not f or len(f) < 10:
            continue
        try:
            lat_raw, ns, lon_raw, ew, alt = f[1], f[2], f[3], f[4], f[8]
            if not lat_raw or not lon_raw or not alt:
                continue
            lat = (int(lat_raw[:2]) + float(lat_raw[2:]) / 60.0) * (1 if ns == "N" else -1)
            lon = (int(lon_raw[:3]) + float(lon_raw[3:]) / 60.0) * (1 if ew == "E" else -1)
            gga.append((t, lat, lon, float(alt)))
        except (ValueError, IndexError):
            continue
    elif "PPSGEN" in line:
        def g(key):
            mm = re.search(rf"\b{key}=(-?\d+)\b", line)
            return int(mm.group(1)) if mm else None
        hw, lk = g("hwphase_ns"), g("lk")
        if hw is None or lk is None:
            continue
        pps.append((t, hw, lk, g("slope_mppb") or 0, g("trim_mppb") or 0,
                    (g("interval_ns") or 1_000_000_000) - 1_000_000_000))

if len(gga) < 100 or len(pps) < 100:
    sys.exit(f"need ≥100 each; got GGA={len(gga)} PPSGEN={len(pps)}")

# --- 位置を平均からの偏差(meter)へ。絶対座標は内部のみ、出力しない ---
lat0 = np.mean([x[1] for x in gga])
lon0 = np.mean([x[2] for x in gga])
alt0 = np.mean([x[3] for x in gga])
mperdeg = 111_320.0
coslat = math.cos(math.radians(lat0))
gga_t = np.array([x[0] for x in gga])
dN = np.array([(x[1] - lat0) * mperdeg for x in gga])         # 北偏差 m
dE = np.array([(x[2] - lon0) * mperdeg * coslat for x in gga])  # 東偏差 m
dU = np.array([x[3] - alt0 for x in gga])                       # 鉛直偏差 m

# --- PPSGEN を locked かつ非異常(|hw|<5µs)に絞り、各々を最近接 GGA に対応づける ---
rows = [(t, hw, sl, tr, dv) for (t, hw, lk, sl, tr, dv) in pps if lk == 1 and abs(hw) < 5000]
P_t = np.array([r[0] for r in rows])
hw = np.array([r[1] for r in rows], float)
slope = np.array([r[2] for r in rows], float)
trim = np.array([r[3] for r in rows], float)

idx = np.searchsorted(gga_t, P_t)
idx = np.clip(idx, 1, len(gga_t) - 1)
# 左右の近い方
left = gga_t[idx - 1]
right = gga_t[idx]
use = np.where(np.abs(P_t - left) <= np.abs(P_t - right), idx - 1, idx)
dt = np.abs(P_t - gga_t[use])
ok = dt < 0.6  # 0.6s 以内で対応づく分だけ
pN, pE, pU = dN[use][ok], dE[use][ok], dU[use][ok]
H = hw[ok]; SL = slope[ok]; TR = trim[ok]
n = ok.sum()
if n < 100:
    sys.exit(f"only {n} aligned locked samples")

span_min = (P_t[-1] - P_t[0]) / 60.0
print(f"# {LOG}: {n} aligned locked samples, span {span_min:.1f} min, mean align dt {dt[ok].mean()*1000:.0f}ms")
print(f"# 位置偏差 RMS [m]: 北={pN.std():.2f}  東={pE.std():.2f}  鉛直={pU.std():.2f}  "
      f"水平={np.hypot(pN,pE).std():.2f}  (座標は非出力)")
print(f"# 制御量: hwphase σ={H.std():.0f}ns  slope σ={SL.std():.0f}mppb  trim σ={TR.std():.0f}mppb")


def corr(a, b):
    if a.std() < 1e-9 or b.std() < 1e-9:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def lp(x, w):
    if len(x) < w:
        return x - x.mean()
    k = np.ones(w) / w
    return np.convolve(x - x.mean(), k, mode="same")


pH = np.hypot(pN, pE)
print("\n# 相関 r(位置偏差, 制御量)  [|r|大=結合あり。鉛直×時刻が最強の理論予測]")
print(f"{'':>14}{'hwphase':>9}{'slope':>9}{'trim':>9}")
for label, p in [("鉛直 dU", pU), ("水平 dH", pH), ("北 dN", pN), ("東 dE", pE)]:
    print(f"{label:>14}{corr(p,H):>9.2f}{corr(p,SL):>9.2f}{corr(p,TR):>9.2f}")

# 遅い成分(30s 移動平均)= position-time coupling は低周波。ループが高域を消すのでここが本命。
print(f"\n# 遅い成分のみ({SLOW_WIN}s 移動平均後)の相関:")
HL, SLL, TRL = lp(H, SLOW_WIN), lp(SL, SLOW_WIN), lp(TR, SLOW_WIN)
print(f"{'':>14}{'hwphase':>9}{'slope':>9}{'trim':>9}")
for label, p in [("鉛直 dU", pU), ("水平 dH", pH)]:
    pl = lp(p, SLOW_WIN)
    print(f"{label:>14}{corr(pl,HL):>9.2f}{corr(pl,SLL):>9.2f}{corr(pl,TRL):>9.2f}")

# ラグ相関: 鉛直偏差(遅い)と hwphase(遅い)。位置と時刻が同根なら lag≈0 にピーク。
dUL = lp(pU, SLOW_WIN)
best_r, best_lag = 0.0, 0
print(f"\n# ラグ相関 r(鉛直偏差[遅], hwphase[遅] を kラグ): k=-{MAXLAG}..{MAXLAG}s")
line = []
for k in range(-MAXLAG, MAXLAG + 1, 4):
    if k < 0:
        r = corr(dUL[:k], HL[-k:])
    elif k > 0:
        r = corr(dUL[k:], HL[:-k])
    else:
        r = corr(dUL, HL)
    line.append(f"{k:+d}:{r:+.2f}")
    if abs(r) > abs(best_r):
        best_r, best_lag = r, k
print("  " + "  ".join(line))
print(f"\n# 解釈: 鉛直×(hwphase/slope/trim) の |r| が有意(≳0.3)なら position-time coupling が実在し、"
      f"位置を固定(survey-in/position-hold)すれば時刻 wander を断てる見込み。最強ラグ k={best_lag}s r={best_r:+.2f}。")
print("# |r| が小さければ coupling 仮説は弱く、別の支配源(熱・受信機内部処理)を疑う。")
