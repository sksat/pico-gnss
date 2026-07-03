#!/usr/bin/env python3
"""scope_autoscale.py — 2本の 1PPS エッジ(CH1=GPS, CH2=gen)の offset と wander を
自己計測し、横軸(timebase)を「エッジが分離して見え、かつ wander でも画面内に留まる」
適切なスケールに自動設定する。

横軸が広すぎると offset(数十 ns)が 1 div 未満で 2 エッジが重なる。狭すぎると wander
(出力位相の数百 ns のゆらぎ)でエッジが shot ごとに画面外へ出る。両者の実測から
ちょうどよい s/div を選ぶ。CH1=1×(GPS 直結)/CH2=10×(プローブ)も毎回ここで正す。

使い方:
  python docs/report/scope_autoscale.py [mode] [N]
    mode = live   (既定) … wander を画面内に保ちつつ最大ズーム。NORM ライブ観測向け
           single        … 瞬時 offset を解像するズーム(:SINGle の静止画向け。ライブだと wander で外れる)
           <s/div>       … 明示指定(例 20e-9)
    N    = offset/wander 計測の shot 数(既定 20)
環境変数: RIGOL_HOST(例 192.168.0.11)
"""
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scope_pps import Rigol, rising_edge

# 物理結線に合わせた既定 setup。CH1=GPS 直結(1×), CH2=gen を 10× プローブ。
# 縦は 0->3.3V が画面に収まる 0.6V/div・offset -2.1V(scope_pps.CAPTURE_SETUP と同条件)。
SETUP = [
    ":CHANnel1:DISPlay 1", ":CHANnel2:DISPlay 1",
    ":CHANnel3:DISPlay 0", ":CHANnel4:DISPlay 0",
    ":CHANnel1:PROBe 1", ":CHANnel2:PROBe 10",
    ":CHANnel1:SCALe 0.6", ":CHANnel2:SCALe 0.6",
    ":CHANnel1:OFFSet -2.1", ":CHANnel2:OFFSet -2.1",
    ":TRIGger:MODE EDGE",
    ":TRIGger:EDGE:SOURce CHANnel1",   # 基準は GPS エッジ
    ":TRIGger:EDGE:SLOPe POSitive",
    ":TRIGger:EDGE:LEVel 1.65",        # 3.3V の中点
    ":TRIGger:SWEep NORMal",           # 1PPS は NORM(AUTO だと未同期になる)
]

DIVS = 10  # DHO800 は横 10 div


STEPS_125 = [1, 2, 5]


def _steps_around(x):
    k = math.floor(math.log10(x))
    out = []
    for kk in (k - 1, k, k + 1, k + 2):
        for m in STEPS_125:
            out.append(m * 10.0 ** kk)
    return sorted(out)


def snap_125_up(x):
    """x [s/div] を 1-2-5 系列の直近上位へ丸める。"""
    if x <= 0:
        return 1e-9
    for v in _steps_around(x):
        if x <= v * (1 + 1e-9):
            return v
    return _steps_around(x)[-1]


def snap_125_near(x):
    """x [s/div] を 1-2-5 系列の最近接へ丸める(ズーム方向に寄せたい時)。"""
    if x <= 0:
        return 1e-9
    return min(_steps_around(x), key=lambda v: abs(v - x))


def measure_offsets(s, n, meas_scale=2e-7):
    """広めの timebase で N shot、GPS->gen の rising offset [s] を集める。"""
    s.send(f":TIMebase:MAIN:SCALe {meas_scale}")
    s.send(":TIMebase:MAIN:OFFSet 0")
    xinc = float(s.query(":WAVeform:XINCrement?"))
    ds = []
    for _ in range(n):
        if not s.single():
            continue
        i1 = rising_edge(s.waveform(1))
        i2 = rising_edge(s.waveform(2))
        if i1 is None or i2 is None:
            continue
        if not (15 < i2 < len(s.waveform(2)) - 15):  # gen が画面端なら不確かなので捨てる
            continue
        ds.append((i2 - i1) * xinc)
    return ds


def framed_fraction(s, shots=5, guard=0.06):
    """現在の設定で shots 回 single し、両エッジが画面内(端から guard 以上)の割合を返す。"""
    ok = 0
    for _ in range(shots):
        if not s.single():
            continue
        w1, w2 = s.waveform(1), s.waveform(2)
        i1, i2, nn = rising_edge(w1), rising_edge(w2), len(w2)
        if i1 is None or i2 is None:
            continue
        if guard * nn < i1 < (1 - guard) * nn and guard * nn < i2 < (1 - guard) * nn:
            ok += 1
    return ok / shots


def _ch1_edge_at(s, off):
    """OFFS=off を設定して 1 shot、CH1(トリガ)エッジのサンプル位置と全長を返す。"""
    s.send(f":TIMebase:MAIN:OFFSet {off}")
    if not s.single():
        return None, 0
    w1 = s.waveform(1)
    return rising_edge(w1), len(w1)


def center_trigger(s, scale):
    """CH1(トリガ)エッジを画面中央に置く OFFS [s] を設定して返す。

    CH1 はトリガなので位置は OFFS だけで決まり wander しない。OFFS→エッジ位置の傾きを
    2 点(0 と 1 div)で実測して中央 n/2 に解く。:TIMebase:OFFSet の符号規約に依らない。"""
    i0, n = _ch1_edge_at(s, 0.0)
    i1, _ = _ch1_edge_at(s, scale)  # 1 div ずらす
    if i0 is None or i1 is None or n == 0 or abs(i1 - i0) < 1e-6:
        s.send(":TIMebase:MAIN:OFFSet 0")
        return 0.0
    slope = (i1 - i0) / scale            # [samples / s of OFFS]
    off = (n / 2.0 - i0) / slope
    s.send(f":TIMebase:MAIN:OFFSet {off}")
    return off


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "live"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 20

    with Rigol() as s:
        s.drain_errors()
        for c in SETUP:
            s.send(c)

        ds = measure_offsets(s, n)
        if len(ds) < 3:
            print(f"エッジ対が少なすぎ ({len(ds)}/{n})。信号/トリガ/プローブを確認。")
            return
        mu = statistics.mean(ds)
        sd = statistics.pstdev(ds)
        pp = max(ds) - min(ds)
        print(f"計測: N_ok={len(ds)}/{n}  mean={mu*1e9:.0f}ns  sigma={sd*1e9:.1f}ns  "
              f"pp={pp*1e9:.0f}ns  (min={min(ds)*1e9:.0f} max={max(ds)*1e9:.0f})")

        # live: offset を ~2-3 div に分離し ±1σ の wander を見せる。±2σ の裾は端で切れてよい
        # (切れること自体が wander の可視化になる)。最近接丸めでズーム側に寄せる。
        live = min(max(snap_125_near(max(abs(mu), 2 * sd, 20e-9) / 2.5), 1e-8), 1e-6)
        # single: 瞬時 offset を ~2 div で解像(:SINGle 静止画向け。ライブだと wander で外れる)。
        single = min(max(snap_125_near(max(abs(mu), 8e-9) / 2), 2e-9), 2e-7)
        print(f"候補: live(offset 分離+±1σ)={live*1e9:.0f}ns/div   "
              f"single(瞬時 offset 解像)={single*1e9:.0f}ns/div")

        try:
            scale = snap_125_up(float(mode))
            label = f"forced {scale*1e9:.0f}ns/div"
        except ValueError:
            scale = single if mode == "single" else live
            label = mode

        # CH1(トリガ)を画面中央に置き(OFFS=0)、スケールだけ自動。wander でエッジが外れる
        # ようなら 1-2-5 ステップで広げて収める(最大数段)。
        # CH1(トリガ)を画面中央に置き、CH2 が wander で外れるならスケールを広げて収める。
        off = 0.0
        for _ in range(3):
            s.send(f":TIMebase:MAIN:SCALe {scale}")
            off = center_trigger(s, scale)        # CH1 トリガを中央へ
            if framed_fraction(s) >= 0.6:          # CH2 の ±2σ 裾は切れてよい
                break
            scale = snap_125_up(scale * 1.5)

        rb = float(s.query(":TIMebase:MAIN:SCALe?"))
        print(f"設定: {label} -> {rb*1e9:.0f}ns/div  CH1 中央(offset={off*1e9:.0f}ns)  "
              f"(span≈{rb*DIVS*1e9:.0f}ns, 横{DIVS}div)")

        # 代表画: 両エッジが収まった 1 shot を選んでスクショ(wander で CH2 が外れた瞬間を避ける)。
        for _ in range(12):
            if not s.single():
                continue
            w1, w2 = s.waveform(1), s.waveform(2)
            i1, i2, nn = rising_edge(w1), rising_edge(w2), len(w2)
            if i1 is not None and i2 is not None and 0.06 * nn < i1 < 0.94 * nn and 0.06 * nn < i2 < 0.94 * nn:
                break
        s.screenshot(os.path.join(os.path.dirname(os.path.abspath(__file__)), "scope-autoscale.png"))
        s.send(":RUN")  # 最後はライブ(NORM)へ戻す
        print("保存: docs/report/scope-autoscale.png  errors:", s.drain_errors())


if __name__ == "__main__":
    main()
