#!/usr/bin/env python3
"""recal あり/なし比較実験のチェックポイント計測 (VXI-11 版。raw socket が死んでいる session 用)。
1 回の呼び出しで: N shot の ch2-ch1 エッジ時間差の統計 + 最終フレームのスクショ + results.csv 追記。
全チェックポイントで同じ timebase (100 ns/div) を使い、スクショを正直に比較できるようにする。
usage: RIGOL_HOST=... uv run --python 3.12 --with python-vxi11 python3 checkpoint.py <tag> [N]
"""
import os, sys, time, statistics as st
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "scripts"))
if os.environ.get("SCOPE_TRANSPORT") == "raw":
    from rigol_raw import RigolRaw as RigolVxi11  # VXI-11 read 死亡時のフォールバック
else:
    from rigol_vxi11 import RigolVxi11

SDIV = 100e-9  # 100 ns/div 固定 (全チェックポイント共通)


def rising_edge(w):
    lo, hi = min(w), max(w)
    if hi - lo < 30:
        return None
    th = (lo + hi) / 2
    for i in range(1, len(w)):
        if w[i - 1] < th <= w[i]:
            return (i - 1) + (th - w[i - 1]) / (w[i] - w[i - 1])
    return None


def setup(r):
    for cmd in (
        ":CHANnel1:PROBe 1", ":CHANnel2:PROBe 10",
        ":CHANnel1:DISPlay ON", ":CHANnel1:SCALe 1", ":CHANnel1:OFFSet -1.5",
        ":CHANnel2:DISPlay ON", ":CHANnel2:SCALe 1", ":CHANnel2:OFFSet -1.5",
        ":CHANnel3:DISPlay OFF",
        ":TRIGger:EDGE:SOURce CHANnel1", ":TRIGger:EDGE:LEVel 1.65", ":TRIGger:EDGE:SLOPe POSitive",
        f":TIMebase:MAIN:SCALe {SDIV:.3e}", ":TIMebase:MAIN:OFFSet 0",
        ":TRIGger:SWEep SINGle",
    ):
        r.send(cmd)
    time.sleep(0.4)
    r.drain_errors()


def one_shot(r, xinc_ns):
    if not r.single(settle=0.1, tries=30):
        return None
    w1 = list(r.waveform(1))
    e1 = rising_edge(w1)
    if e1 is None:
        return None
    w2 = list(r.waveform(2))
    e2 = rising_edge(w2)
    if e2 is None:
        return None
    return (e2 - e1) * xinc_ns


def main():
    tag = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    r = RigolVxi11(timeout=8.0)
    r.clear()
    setup(r)
    xinc_ns = float(r.query(":WAVeform:XINCrement?")) * 1e9
    offs = []
    fails = 0
    while len(offs) < n and fails < n:
        try:
            v = one_shot(r, xinc_ns)
        except Exception:
            fails += 1
            try:
                r.clear(); setup(r)
            except Exception:
                time.sleep(2)
            continue
        if v is None:
            fails += 1
        else:
            offs.append(v)
            with open(os.path.join(HERE, f"{tag}.shots"), "a") as f:
                f.write(f"{time.time():.1f} {v:.1f}\n")
    png = os.path.join(HERE, f"{tag}.png")
    try:
        r.screenshot(png)
    except Exception as e:
        print("screenshot failed:", e)
    m = st.mean(offs) if offs else float("nan")
    s = st.pstdev(offs) if len(offs) > 1 else float("nan")
    line = (f"{time.strftime('%H:%M:%S')} {tag} n={len(offs)}/{n + fails} xinc={xinc_ns:.2f}ns "
            f"mean={m:+.1f}ns sigma={s:.1f}ns min={min(offs):+.1f} max={max(offs):+.1f}" if offs
            else f"{time.strftime('%H:%M:%S')} {tag} NO DATA (fails={fails})")
    with open(os.path.join(HERE, "results.txt"), "a") as f:
        f.write(line + "\n")
    print(line)
    # 合間は RUN (NORMal) に戻して画面をライブに保つ (次の checkpoint の setup が SINGle に戻す)
    try:
        r.send(":TRIGger:SWEep NORMal")
        r.send(":RUN")
    except Exception:
        pass
    r.close()


if __name__ == "__main__":
    main()
