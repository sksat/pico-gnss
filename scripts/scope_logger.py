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
import os, sys, time, re, collections, statistics, signal, errno
sys.path.insert(0, ".claude/skills/oscilloscope-timing/scripts")
sys.path.insert(0, "scripts")  # rigol_vxi11 (VXI-11 transport)
from rigol_scpi import Rigol, rising_edge

# トランスポート選択。raw socket(5555)は :WAVeform:DATA? 中断で off-by-one に固着し power-cycle 必須に
# なるので、**SCOPE_TRANSPORT=vxi11** を推奨 (proper RPC framing で off-by-one 不発 + device clear で
# 自己復旧。Python 3.12 以下 + python-vxi11 が要る: uv run --python 3.12 --with python-vxi11 ...)。
SCOPE_TRANSPORT = os.environ.get("SCOPE_TRANSPORT", "raw").lower()

SCOPE_LOG  = os.environ.get("SCOPE_LOG", "logs/scope-edge.log")
FW_LOG     = os.environ.get("FW_LOG", "logs/pps.log")
REF_CH     = int(os.environ.get("REF_CH", "1"))
SIG_CH     = int(os.environ.get("SIG_CH", "2"))
N_SHOTS    = int(os.environ.get("N_SHOTS", "7000"))
SHOT_DIR   = os.environ.get("SHOT_DIR", "logs/scope-shots")
SHOT_EVERY = int(os.environ.get("SHOT_EVERY", "150"))
# 横軸の自動調整 (AUTO_TB=0 で無効)。直近の offset 分布に合わせ ±~3.5σ が画面に収まる s/div へ。
# 罠 #4: 締めすぎると wander でエッジが画面外に出た shot を選択的に落とし σ を下偏させるので、
# ±3.5σ + |median| を ~4 div に収める広めの窓にする (詰めすぎない)。1-2-5 ステップにスナップ。
AUTO_TB       = os.environ.get("AUTO_TB", "1") != "0"
AUTO_TB_EVERY = int(os.environ.get("AUTO_TB_EVERY", "60"))  # 何 shot ごとに見直すか
_TB_STEPS = [n * d for d in (1e-9, 1e-8, 1e-7, 1e-6) for n in (1, 2, 5)]  # 1/2/5 × decade

def choose_timebase_s(offsets_ns):
    """直近 offset の中央 90% (p5..p95) が ~片側4div に収まる最小の 1-2-5 s/div を返す。
    σ でなく percentile で枠取りするのは、発散する制御 slot や pull-in の外れ値で窓が膨らんで
    「変わらない」ように見えるのを防ぐため (中央 90% を基準にしつつ、外れ値はたまに画面外でよい)。"""
    if len(offsets_ns) < 20:
        return None
    s = sorted(offsets_ns)
    n = len(s)
    p5, p95 = s[int(n * 0.05)], s[min(n - 1, int(n * 0.95))]
    half_ns = max(abs(p5), abs(p95)) + 50.0  # +50ns マージン (片側)
    target = (half_ns * 1e-9) / 4.0          # s/div (片側 ~4 div)
    for step in _TB_STEPS:
        if step >= target:
            return step
    return _TB_STEPS[-1]

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

# ── 単一インスタンスロック: 2 つ目のクライアントが scope に触れるのを防ぐ ──────────────────
# **scope は 1 クライアント限定**。2 接続が同時に SCPI を叩くと session が壊れる (off-by-one /
# STOP)。logs/.scope_logger.lock に pid を置き、生きた別インスタンスがあれば起動を拒否する。
# kill -9 等で残った死 pid の残骸は stale とみなし奪取する。終了時 (try/finally) に必ず外す。
LOCK_PATH = os.path.join(os.path.dirname(SCOPE_LOG) or ".", ".scope_logger.lock")

def _pid_alive(pid):
    try:
        os.kill(pid, 0)
    except OSError as e:
        return e.errno == errno.EPERM  # 存在するが別ユーザ所有
    except Exception:
        return False
    return True

def acquire_lock(path):
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    for _ in range(2):
        try:
            fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
            os.write(fd, f"{os.getpid()}\n".encode())
            os.close(fd)
            return
        except FileExistsError:
            try:
                with open(path) as lf:
                    old = int(lf.read().strip())
            except (ValueError, OSError):
                old = None
            if old is not None and old != os.getpid() and _pid_alive(old):
                sys.exit(f"another scope_logger holds {path} (pid {old}); refusing to start. "
                         f"Only ONE client may talk to the scope at a time. "
                         f"If that pid is dead, remove {path} and retry.")
            try:                       # stale (死 pid / 読めない) -> 奪取して retry
                os.remove(path)
            except OSError:
                pass
    sys.exit(f"could not acquire lock {path}")

def release_lock(path):
    try:
        with open(path) as lf:
            if int(lf.read().strip()) != os.getpid():
                return                 # 別インスタンスのロックは消さない
    except (ValueError, OSError):
        return
    try:
        os.remove(path)
    except OSError:
        pass

# ── クリーン停止: SIGTERM/SIGINT で stop flag を立て、in-flight の shot を**完遂**してから止める ──
# 危険なのは :WAVeform:DATA? のバイナリブロック読みの最中に殺され、session が off-by-one になること。
# flag は **shot と shot の間でのみ** 確認する (波形読みの途中では見ない)。ハンドラは flag を立てる
# だけで例外を投げないので、PEP 475 によりブロック中の recv は中断されず**再開**される — よって
# 半端な block を残さない。socket の close は下の try/finally で必ず走る。
_stop = False
def _on_signal(signum, _frame):
    global _stop
    _stop = True
    print(f"\nscope logger: signal {signum} received -> finishing in-flight shot, then stopping.",
          flush=True)
for _sig in (signal.SIGTERM, signal.SIGINT):
    signal.signal(_sig, _on_signal)

# 堅牢な realign: 中断した別クライアントが残したキュー応答を**完全に**排出する。1 クエリ送って
# 「2 秒間 無音」になるまで届く応答を全部読み捨てる (短い timeout だと scope の遅い応答を取り逃し
# 「常に 1 応答遅れ」が残るので、無音検出にする)。さらに既知クエリ (*IDN?) が**自分の**応答を返す
# まで検証つきでリトライし、desync/garbled な初回応答にも耐える。**scope は 1 クライアント限定** —
# このスクリプト稼働中に別接続で scope を query しないこと (socket desync で STOP)。監視はログで行う。
def _drain_quiet(rig, quiet=2.0):
    """届く応答を `quiet` 秒の無音になるまで全部読み捨てる (キューに溜まった stale 応答の排出)。"""
    rig.s.settimeout(quiet)
    last = time.time()
    while time.time() - last < quiet:
        try:
            d = rig.s.recv(4096)
            if not d:
                break
            last = time.time()
        except Exception:
            break
    rig.s.settimeout(rig.timeout)

def _realign(rig, attempts=6, quiet=2.0):
    """off-by-one / 文字化けした SCPI session を resync する。キュー応答を無音まで排出し、
    既知クエリ (*IDN?) が **自分の** 応答を返すまでリトライする (garbled な初回応答に耐える)。"""
    for _ in range(attempts):
        rig.send("*CLS")
        _drain_quiet(rig, quiet)
        try:
            idn = rig.query("*IDN?")
        except Exception:
            idn = ""
        if "RIGOL" in idn.upper() or "DHO" in idn.upper():
            return True
        _drain_quiet(rig, quiet)      # まだズレている -> もう一周排出してリトライ
    return False

acquire_lock(LOCK_PATH)
ok = bad = shots = 0
r = None
try:
    if SCOPE_TRANSPORT == "vxi11":
        from rigol_vxi11 import RigolVxi11
        r = RigolVxi11(timeout=6.0)
        try:
            r.clear()  # VXI-11 device clear: SCPI 状態をリセット (raw-socket の off-by-one を回避)
        except Exception:
            pass
        r.drain_errors()
    else:
        r = Rigol(timeout=6.0)
        _realign(r)
        r.drain_errors()
    # xinc は desync 耐性のためリトライ (失敗時は次の応答を読んで自然に整合)。
    xinc = None
    for _ in range(5):
        try:
            xinc = float(r.query(":WAVeform:XINCrement?")) * 1e9
            break
        except ValueError:
            continue
    if xinc is None:
        _realign(r)
        xinc = float(r.query(":WAVeform:XINCrement?")) * 1e9  # ns/sample (現在の s/div で測る)
    recent = collections.deque(maxlen=max(40, AUTO_TB_EVERY * 2))  # 直近 offset(ns) for auto-timebase
    cur_tb = float(r.query(":TIMebase:MAIN:SCALe?"))
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
            if _stop:                 # stop flag は **shot 間でのみ** 確認 (波形読みの途中では見ない)
                break
            try:
                if not r.single():   # 1PPS は低レートなので NORMal トリガで 1 エッジ待つ
                    bad += 1; continue
                wr = r.waveform(REF_CH); ws = r.waveform(SIG_CH)
                er = rising_edge(wr); es = rising_edge(ws)
                if er is not None and es is not None:
                    wt = time.time()
                    off = (es - er) * xinc
                    f.write(f"{wt:.3f} {off:.1f}\n"); f.flush(); ok += 1; shots += 1
                    recent.append(off)
                    # 横軸の自動調整: 直近分布に合わせ ±~3.5σ が収まる s/div へ (変化時のみ設定 + xinc 再読み)。
                    if AUTO_TB and (shots == 30 or shots % AUTO_TB_EVERY == 0):
                        tb = choose_timebase_s(list(recent))
                        if tb is not None and abs(tb - cur_tb) / cur_tb > 0.01:
                            r.send(f":TIMebase:MAIN:SCALe {tb}")
                            time.sleep(0.2)
                            xinc = float(r.query(":WAVeform:XINCrement?")) * 1e9
                            cur_tb = tb
                            print(f"auto-timebase -> {tb*1e9:.0f} ns/div (xinc={xinc:.2f} ns/pt)", flush=True)
                    # single 直後 = STOP で両エッジ表示中。SHOT_EVERY ごとにスクショ (証跡 / GIF 素材)
                    if SHOT_EVERY > 0 and shots % SHOT_EVERY == 0:
                        try:
                            r.screenshot(os.path.join(SHOT_DIR, f"scope-{wt:.0f}.png"))
                        except Exception:
                            r.drain_errors()
                else:
                    bad += 1
            except Exception:
                bad += 1; time.sleep(1)
finally:
    if r is not None:
        try:
            r.close()
        except Exception:
            pass
    release_lock(LOCK_PATH)
print(f"scope logger done: ok={ok} bad={bad} -> {SCOPE_LOG} (append={append}), shots in {SHOT_DIR}")
