#!/usr/bin/env python3
"""出力 1PPS と GPS PPS の立ち上がりエッジ差 (output-GPS) を Rigol DHO800 で連続記録する。

oscilloscope-timing skill の `measure_edge_offset` が「密に多 shot して分布で報告」するのに対し、
こちらは **1 shot ごとに wall-time タイムスタンプを付けて延々と流す** ためのもの。firmware の
hwphase (= PIO で測る同じ output-GPS) と時刻で突き合わせ、第3計器として独立に裏取りするのに使う。

なぜ wall-time を打つか: ホストが NTP 同期していれば `time.time()` は絶対 UTC なので、scope が
捉えたパルスがどの UTC 秒・どの firmware エッジかを後から対応づけられる (`timedatectl` で
synchronized を確認すること)。冒頭に firmware ログの最新 RTT 時刻と wall を 1 行記録しておくと、
firmware の RTT 時計と wall の橋になり、相互相関でラグを較正できる (analyze 側でやる)。

接続先 scope の IP は `RIGOL_HOST` env から取る (機器の所在は直書きしない。skill 参照)。
実行は repo ルートから (skill の rigol_scpi.py を相対 import するため)。

env:
  RIGOL_HOST  scope IP (必須)
  SCOPE_LOG   出力ログ (既定 logs/scope-edge.log)
  FW_LOG      RTT 橋に使う firmware ログ (既定 logs/pps.log。無ければ橋なしで wall のみ記録)
  REF_CH SIG_CH  基準/信号チャネル (既定 1=GPS, 2=出力)。offset = SIG-REF。
  N_SHOTS     最大 shot 数 (既定 7000 ≒ 1Hz で ~2h)
"""
import os, sys, time, re
sys.path.insert(0, ".claude/skills/oscilloscope-timing/scripts")
from rigol_scpi import Rigol, rising_edge

SCOPE_LOG = os.environ.get("SCOPE_LOG", "logs/scope-edge.log")
FW_LOG    = os.environ.get("FW_LOG", "logs/pps.log")
REF_CH    = int(os.environ.get("REF_CH", "1"))
SIG_CH    = int(os.environ.get("SIG_CH", "2"))
N_SHOTS   = int(os.environ.get("N_SHOTS", "7000"))

# RTT↔wall 橋: firmware ログの最新 RTT 時刻と現 wall を 1 行に残す (後で scope shot を
# firmware エッジ・cidx 区間へ対応づけるため)。FW_LOG が無ければ橋なし。
rtt0 = None
try:
    for l in open(FW_LOG, errors="ignore"):
        m = re.match(r"\s*([\d.]+)\s", l)
        if m and "PPSGEN" in l:
            try: rtt0 = float(m.group(1))
            except ValueError: pass
except FileNotFoundError:
    pass
wall0 = time.time()

r = Rigol(); r.drain_errors()
xinc = float(r.query(":WAVeform:XINCrement?")) * 1e9  # ns/sample (現在の s/div で測る)
ok = bad = 0
with open(SCOPE_LOG, "w") as f:
    if rtt0 is not None:
        f.write(f"# wall0={wall0:.3f} rtt0={rtt0:.3f}  -> RTT = wall - {wall0-rtt0:.3f}; "
                f"cols: wall_time offset_ns(ch{SIG_CH}-ch{REF_CH})\n")
    else:
        f.write(f"# wall0={wall0:.3f} (RTT 橋なし: FW_LOG 未取得); cols: wall_time offset_ns(ch{SIG_CH}-ch{REF_CH})\n")
    f.flush()
    for _ in range(N_SHOTS):
        try:
            if not r.single():   # 1PPS は低レートなので NORMal トリガで 1 エッジ待つ
                bad += 1; continue
            wr = r.waveform(REF_CH); ws = r.waveform(SIG_CH)
            er = rising_edge(wr); es = rising_edge(ws)
            if er is not None and es is not None:
                f.write(f"{time.time():.3f} {(es-er)*xinc:.1f}\n"); f.flush(); ok += 1
            else:
                bad += 1
        except Exception:
            bad += 1; time.sleep(1)
print(f"scope logger done: ok={ok} bad={bad} -> {SCOPE_LOG}")
