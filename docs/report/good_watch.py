#!/usr/bin/env python3
"""良受信を検知したら「定義合わせ」実測 (defmatch.py) を自動実行するウォッチャー。

定義合わせの threshold スイープは「GP3 が GPS 近傍 = 計測しやすい」良受信時に走らせたい
(弱受信/holdover だと GP3 が GPS から数十µs 離れる)。lock 維持・holdover 小・衛星多・
HDOP 低・|hwphase| 小 が QWIN 秒続いたら発火し、同じディレクトリの defmatch.py を実行、
結果を保存して終了 (= 親プロセスに通知)。

  RIGOL_HOST=<scope-ip> python3 docs/report/good_watch.py
env:
  GNSS_LOG        firmware の defmt ログ (default /tmp/pps-flash.log)
  DEFMATCH_RESULT 結果保存先 (default /tmp/defmatch-result.txt)
  RIGOL_HOST      scope SCPI ホスト (defmatch.py が使用)
"""
import re, sys, os, time, collections, subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.environ.get("GNSS_LOG", "/tmp/pps-flash.log")
RESULT = os.environ.get("DEFMATCH_RESULT", "/tmp/defmatch-result.txt")

T_RE    = re.compile(r"^(\d+\.\d+)\s")
HW_RE   = re.compile(r"hwphase_ns=(-?\d+)")
TIME_RE = re.compile(r"TIME .*holdover_ms=(\d+) locked=(\d+)")
GGA_RE  = re.compile(r"\$G[A-Z]GGA,([^*]*)")

Q_WIN   = 20.0   # s: 良受信が続くべき窓
HDOP_OK = 1.3    # 中央値 HDOP <= これ
SATS_OK = 8      # 使用衛星 >= これ
HOLD_OK = 500    # holdover_ms < これ (PPS 生存)
HW_OK   = 500    # |hwphase_ns| < これ (GP3 が GPS 近傍 = 窓内)
SUSTAIN = 0.8    # 窓内サンプルのこの割合以上が良であること

hw   = collections.deque()
hold = collections.deque()
q    = collections.deque()


def trim(dq, now):
    while dq and now - dq[0][0] > Q_WIN:
        dq.popleft()


def follow(path):
    f = open(path, "r"); f.seek(0, 2)
    while True:
        line = f.readline()
        if not line:
            time.sleep(0.5); continue
        yield line


def run_defmatch():
    if not os.environ.get("RIGOL_HOST"):
        print("RIGOL_HOST unset; skip defmatch", flush=True); return
    print("good reception -> running defmatch.py", flush=True)
    try:
        r = subprocess.run([sys.executable, os.path.join(HERE, "defmatch.py")],
                           cwd=HERE, env=os.environ.copy(),
                           capture_output=True, text=True, timeout=240)
        out = r.stdout + ("\nSTDERR:\n" + r.stderr if r.returncode else "")
        with open(RESULT, "w") as f:
            f.write(out)
        print(out, flush=True)
    except Exception as e:
        print("defmatch failed:", repr(e), flush=True)


def main():
    print("good-reception watcher armed (HDOP<=%.1f, sats>=%d, holdover<%dms, |hwphase|<%dns, %.0fs)"
          % (HDOP_OK, SATS_OK, HOLD_OK, HW_OK, Q_WIN), flush=True)
    last_hb = 0.0
    for line in follow(LOG):
        m = T_RE.match(line)
        if not m:
            continue
        t = float(m.group(1))

        mh = HW_RE.search(line)
        if mh:
            hw.append((t, int(mh.group(1)))); trim(hw, t)
        mt = TIME_RE.search(line)
        if mt:
            hold.append((t, int(mt.group(1)), int(mt.group(2)))); trim(hold, t)
        mg = GGA_RE.search(line)
        if mg:
            f = mg.group(1).split(",")
            try:
                q.append((t, float(f[7]), int(f[6]))); trim(q, t)
            except (ValueError, IndexError):
                pass

        if t - last_hb >= 60:
            last_hb = t
            hd = sorted(h for _, h, _ in q)
            print("[hb t=%.0f] medHDOP=%.2f minSats=%s |hw|max=%dns holdmax=%dms n=%d"
                  % (t, (hd[len(hd)//2] if hd else 0),
                     (min(s for _, _, s in q) if q else "-"),
                     max((abs(v) for _, v in hw), default=0),
                     max((h for _, h, _ in hold), default=0), len(q)), flush=True)

        if len(q) < 12 or len(hold) < 12 or len(hw) < 12:
            continue
        hd = sorted(h for _, h, _ in q)
        med_hdop = hd[len(hd)//2]
        min_sats = min(s for _, _, s in q)
        good_q = sum(1 for _, h, s in q if h <= HDOP_OK and s >= SATS_OK)
        good_hold = all(hm < HOLD_OK and lk == 1 for _, hm, lk in hold)
        hw_ok = all(abs(v) < HW_OK for _, v in hw)
        if (good_q >= SUSTAIN * len(q) and med_hdop <= HDOP_OK and min_sats >= SATS_OK
                and good_hold and hw_ok):
            print("GOOD_RECEPTION t=%.0f medHDOP=%.2f minSats=%d |hw|max=%dns"
                  % (t, med_hdop, min_sats, max(abs(v) for _, v in hw)), flush=True)
            run_defmatch()
            return


if __name__ == "__main__":
    main()
