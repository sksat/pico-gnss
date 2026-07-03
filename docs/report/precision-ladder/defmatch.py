#!/usr/bin/env python3
"""定義合わせ実測: GP3↔GPS の立ち上がり時刻差が、計測しきい値でどれだけ動くか。

RP2040 datasheet Table 625 (@3.3V): V_IH=2.0V(min), V_IL=0.8V(max), hyst>=0.2V。
スイッチ点 typ は未規定なので「0.8-2.0V の帯」= 3.3V swing の約 24-61%。
プローブ比に依存しないよう各 ch を自分の swing(10/90 percentile)で正規化し、
しきい値を swing 割合で振る。同一取得の同一波形対(=wander と分離)。

修正: GP3 は scope 上では GPS から数µs〜数十µs 離れていることがある(holdover 復帰の
過渡や経路オフセット)。まず広い窓で GP3 の位置を探し、両エッジが入るよう窓を合わせて
から(timebase offset)細かくスイープする。RAW で分解能確保。
"""
import os
import sys
# scope_pps は再利用ツールとして scripts/ に移動済み (このスクリプトはレポート専用なので report/ に残す)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "scripts"))
from scope_pps import Rigol

SWING = 3.3
LEVELS = [(0.8, 0.8 / SWING), (1.0, 1.0 / SWING), (1.4, 1.4 / SWING),
          (1.65, 1.65 / SWING), (1.8, 1.8 / SWING), (2.0, 2.0 / SWING)]
N = 12
COARSE_TB = 1e-5  # 10us/div -> +-50us、GP3 を確実に捕捉


def pct(s, p):
    return s[max(0, min(len(s) - 1, int(p * (len(s) - 1))))]


def cross_time(v, xinc, xor, frac):
    if len(v) < 2:
        return None
    s = sorted(v)
    lo, hi = pct(s, 0.10), pct(s, 0.90)
    if hi - lo < 0.3:
        return None
    thr = lo + frac * (hi - lo)
    for i in range(1, len(v)):
        if v[i - 1] < thr <= v[i]:
            f = (thr - v[i - 1]) / (v[i] - v[i - 1])
            return xor + (i - 1 + f) * xinc
    return None


def grab(scope, ch):
    scope.send(f":WAVeform:SOURce CHANnel{ch}")
    scope.send(":WAVeform:MODE RAW")
    scope.send(":WAVeform:FORMat BYTE")
    yinc = float(scope.query(":WAVeform:YINCrement?"))
    yor = float(scope.query(":WAVeform:YORigin?"))
    yref = float(scope.query(":WAVeform:YREFerence?"))
    xinc = float(scope.query(":WAVeform:XINCrement?"))
    xor = float(scope.query(":WAVeform:XORigin?"))
    data = scope.query_block(":WAVeform:DATA?")
    return [(b - yref - yor) * yinc for b in data], xinc, xor


def shot(scope):
    scope.single()
    v1, xinc, xor = grab(scope, 1)  # GPS
    v2, _, _ = grab(scope, 2)       # GP3
    return v1, v2, xinc, xor


with Rigol() as scope:
    for c in (
        ":CHANnel1:DISPlay 1", ":CHANnel2:DISPlay 1",
        ":CHANnel1:PROBe 1", ":CHANnel2:PROBe 10",
        ":CHANnel1:SCALe 0.6", ":CHANnel2:SCALe 0.6",
        ":CHANnel1:OFFSet -2.1", ":CHANnel2:OFFSet -2.1",
        ":TRIGger:EDGE:SOURce CHANnel1", ":TRIGger:EDGE:SLOPe POSitive",
        ":TRIGger:EDGE:LEVel 1.65", ":TRIGger:SWEep NORMal",
    ):
        scope.send(c)
    scope.drain_errors()

    # --- coarse: 広い窓で GP3 の位置(GPS との差)を探す ---
    scope.send(f":TIMebase:MAIN:SCALe {COARSE_TB}")
    scope.send(":TIMebase:MAIN:OFFSet 0")
    coarse = []
    for _ in range(6):
        v1, v2, xinc, xor = shot(scope)
        t1 = cross_time(v1, xinc, xor, 0.5)
        t2 = cross_time(v2, xinc, xor, 0.5)
        if t1 is not None and t2 is not None:
            coarse.append(t2 - t1)
    if not coarse:
        print("GP3 edge not found within +-50us; abort", flush=True)
        scope.send(":TRIGger:SWEep AUTO")
        sys.exit(1)
    coarse.sort()
    coff = coarse[len(coarse) // 2]  # median offset (s)
    print(f"coarse GP3-GPS offset = {coff*1e9:.0f} ns (n={len(coarse)})", flush=True)

    # --- fine: 両エッジが入るよう窓を合わせる ---
    fine_tb = min(1e-5, max(2e-7, abs(coff) / 4))
    scope.send(f":TIMebase:MAIN:SCALe {fine_tb}")
    scope.send(f":TIMebase:MAIN:OFFSet {coff/2}")  # 中点を画面中央へ

    offs = {lv: [] for lv, _ in LEVELS}
    grabbed = 0
    for _ in range(N):
        v1, v2, xinc, xor = shot(scope)
        ok = False
        for lv, frac in LEVELS:
            t1 = cross_time(v1, xinc, xor, frac)
            t2 = cross_time(v2, xinc, xor, frac)
            if t1 is not None and t2 is not None:
                offs[lv].append((t2 - t1) * 1e9)
                ok = True
        if ok:
            grabbed += 1

    scope.send(":TRIGger:SWEep AUTO")
    print(f"captures used: {grabbed}/{N}, fine timebase {fine_tb*1e9:.0f}ns/div", flush=True)
    rows = {lv: sum(xs) / len(xs) for lv, xs in offs.items() if xs}
    base = rows.get(1.65)
    print("equiv_V(of 3.3V swing)  mean_offset(ns)  vs_1.65V(ns)  n")
    for lv, frac in LEVELS:
        if lv in rows:
            d = (rows[lv] - base) if base is not None else float("nan")
            print(f"  {lv:5.2f} ({frac*100:4.1f}%)        {rows[lv]:+9.1f}        {d:+7.1f}     {len(offs[lv])}")
    if rows:
        band = list(rows.values())
        print(f"offset span over 0.8-2.0V band: {max(band)-min(band):.1f} ns")
