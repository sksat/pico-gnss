#!/usr/bin/env python3
"""出力 1PPS と GPS PPS の立ち上がりエッジ差 (output-GPS) を Rigol DHO800 で連続記録する。

oscilloscope-timing skill の `measure_edge_offset` が「密に多 shot して分布で報告」するのに対し、
こちらは **1 shot ごとに wall-time タイムスタンプを付けて延々と流す** ためのもの。firmware の
hwphase (= PIO で測る同じ output-GPS) と時刻で突き合わせ、第3計器として独立に裏取りするのに使う。

なぜ wall-time を打つか: ホストが NTP 同期していれば `time.time()` は絶対 UTC なので、scope が
捉えたパルスがどの UTC 秒・どの firmware エッジかを後から対応づけられる (`timedatectl` で
synchronized を確認すること)。冒頭に firmware ログの最新 RTT 時刻と wall を 1 行記録しておくと、
firmware の RTT 時計と wall の橋になり、相互相関でラグを較正できる (analyze 側でやる)。

途中で止めて再開しても良いように append 対応: SCOPE_LOG に既存の橋ヘッダがあれば再利用して
追記する (firmware を再フラッシュしていない限り橋定数 wall-RTT は連続なので有効)。

SHOT_EVERY shot ごとに **スクショも保存**する。single 直後の STOP 状態 = ちょうど捕捉した両エッジが
出ているので、波形ビューを潰す測定 (Result) ウィンドウは事前に閉じておくこと (`:MEASure:CLEar`)。
ファイル名に wall-time を入れるので、後から config 区間・UTC 秒に対応づけられる。

**タイムベースは wander の全振幅を画面内に収めること。** offset は trigger 中央エッジとの差なので、
振れた瞬間に信号エッジが画面外へ出ると rising_edge が失敗して shot が落ちる。これは単なる取りこぼし
でなく、**大きく振れた瞬間だけ捨てるため scope σ を下に偏らせる** (実際 50ns/div=±250ns 窓で
σ が過小に出た)。±3σ が収まる s/div を選ぶ (例: hwphase σ≈200ns なら ±600ns を収める 200ns/div)。
s/div は logger 側で固定せず現在値の XINCrement をそのまま使う (skill の作法) ので、起動前に scope 側で
設定しておく。

接続先 scope の IP は `RIGOL_HOST` env から取る (機器の所在は直書きしない)。実行は repo ルートから。

env:
  RIGOL_HOST  scope IP (必須)
  SCOPE_LOG   出力ログ (既定 logs/scope-edge.log)。既存なら橋を再利用して追記。
  FW_LOG      RTT 橋に使う firmware ログ (既定 logs/pps.log。無ければ橋なしで wall のみ)
  REF_CH SIG_CH  基準/信号チャネル (既定 1=GPS, 2=出力)。offset = SIG-REF。
  N_SHOTS     最大 shot 数 (既定 7000 ≒ 1Hz で ~2h)
  SHOT_DIR    スクショ保存先 (既定 logs/scope-shots。/logs/* は gitignore 済)
  SHOT_EVERY  何 shot ごとにスクショするか (既定 150 ≒ ~3-5 分。0 で無効)
"""
import os, sys, time, re
sys.path.insert(0, ".claude/skills/oscilloscope-timing/scripts")
from rigol_scpi import Rigol, rising_edge

SCOPE_LOG  = os.environ.get("SCOPE_LOG", "logs/scope-edge.log")
FW_LOG     = os.environ.get("FW_LOG", "logs/pps.log")
REF_CH     = int(os.environ.get("REF_CH", "1"))
SIG_CH     = int(os.environ.get("SIG_CH", "2"))
N_SHOTS    = int(os.environ.get("N_SHOTS", "7000"))
SHOT_DIR   = os.environ.get("SHOT_DIR", "logs/scope-shots")
SHOT_EVERY = int(os.environ.get("SHOT_EVERY", "150"))

# 既存ログの橋ヘッダを探す (append 再開用)。あれば橋定数を再利用、無ければ新規に張る。
bridge_const = None
if os.path.exists(SCOPE_LOG):
    with open(SCOPE_LOG, errors="ignore") as f:
        for l in f:
            m = re.search(r"RTT = wall - ([\d.]+)", l)
            if m:
                bridge_const = float(m.group(1)); break
            if not l.startswith("#"):
                break
append = bridge_const is not None
rtt0 = wall0 = None
if not append:
    try:
        for l in open(FW_LOG, errors="ignore"):
            m = re.match(r"\s*([\d.]+)\s", l)
            if m and "PPSGEN" in l:
                try: rtt0 = float(m.group(1))
                except ValueError: pass
    except FileNotFoundError:
        pass
    wall0 = time.time()
    bridge_const = (wall0 - rtt0) if rtt0 is not None else None

if SHOT_EVERY > 0:
    os.makedirs(SHOT_DIR, exist_ok=True)

r = Rigol(); r.drain_errors()
xinc = float(r.query(":WAVeform:XINCrement?")) * 1e9  # ns/sample (現在の s/div で測る)
ok = bad = shots = 0
with open(SCOPE_LOG, "a" if append else "w") as f:
    if not append:
        if bridge_const is not None:
            f.write(f"# wall0={wall0:.3f} rtt0={rtt0:.3f}  -> RTT = wall - {bridge_const:.3f}; "
                    f"cols: wall_time offset_ns(ch{SIG_CH}-ch{REF_CH})\n")
        else:
            f.write(f"# wall0={time.time():.3f} (RTT 橋なし: FW_LOG 未取得); "
                    f"cols: wall_time offset_ns(ch{SIG_CH}-ch{REF_CH})\n")
        f.flush()
    for _ in range(N_SHOTS):
        try:
            if not r.single():   # 1PPS は低レートなので NORMal トリガで 1 エッジ待つ
                bad += 1; continue
            wr = r.waveform(REF_CH); ws = r.waveform(SIG_CH)
            er = rising_edge(wr); es = rising_edge(ws)
            if er is not None and es is not None:
                wt = time.time()
                f.write(f"{wt:.3f} {(es-er)*xinc:.1f}\n"); f.flush(); ok += 1; shots += 1
                # single 直後 = STOP で両エッジ表示中。SHOT_EVERY ごとにスクショ (証跡)
                if SHOT_EVERY > 0 and shots % SHOT_EVERY == 0:
                    try:
                        r.screenshot(os.path.join(SHOT_DIR, f"scope-{wt:.0f}.png"))
                    except Exception:
                        r.drain_errors()
            else:
                bad += 1
        except Exception:
            bad += 1; time.sleep(1)
print(f"scope logger done: ok={ok} bad={bad} -> {SCOPE_LOG} (append={append}), shots in {SHOT_DIR}")
