# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""GPS-R の 1PPS から、NTP フレームが線に出るまでを測る (Rigol DHO800, SCPI over LAN)。

    RIGOL_HOST=<ip> uv run docs/report/ntp-stratum1/logs/paths/scope_tx_phase.py [N] [out.log]

    CH1 = GPS-R の 1PPS。この基板はアクティブ Low なので、秒境界は**立ち下がり**である。
    CH2 = Pico の GP17 (10BASE-T の TX+)。アイドルは 0 V で、16 ms ごとのリンクパルスと、
          1 秒に 1 回の NTP フレームだけが出る。

firmware は「秒境界 + TX_LAG_NS に送出した」と申告している。ここで測るのはその申告が本当かで、
firmware の外から見た唯一の証拠になる。ログだけでは、自分の測定が自分の誤差を含んでいるかを
判定できない。

取り込みは RAW で深く取る。フレームの先頭は 100 ns のビットなので、画面 1000 点では消える。
"""

import os
import socket
import statistics
import sys
import time

HOST = os.environ.get("RIGOL_HOST")
PORT = int(os.environ.get("RIGOL_PORT", "5555"))
WINDOW_S = 500e-6  # 秒境界から 500 us ぶんを見る
DIV = 10
THRESH_V = 1.0  # CH2 の立ち上がり判定。3.3 V ロジックの半分より低めに取る


class Scope:
    def __init__(self):
        self.s = socket.create_connection((HOST, PORT), timeout=10)

    def send(self, cmd: str):
        self.s.sendall((cmd + "\n").encode())

    def query(self, cmd: str) -> str:
        self.send(cmd)
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = self.s.recv(1 << 16)
            if not chunk:
                break
            buf += chunk
        return buf.decode(errors="replace").strip()

    def block(self, cmd: str) -> bytes:
        self.send(cmd)
        head = b""
        while len(head) < 2:
            head += self.s.recv(2 - len(head))
        ndig = int(head[1:2])
        digits = b""
        while len(digits) < ndig:
            digits += self.s.recv(ndig - len(digits))
        need = int(digits)
        data = b""
        while len(data) < need:
            data += self.s.recv(min(1 << 16, need - len(data)))
        self.s.recv(1)  # trailing newline
        return data

    def wave(self, ch: int):
        self.send(f":WAV:SOUR CHAN{ch}")
        self.send(":WAV:MODE RAW")
        self.send(":WAV:FORM BYTE")
        pre = self.query(":WAV:PRE?").split(",")
        points = int(pre[2])
        xinc, xorig = float(pre[4]), float(pre[5])
        yinc, yorig, yref = float(pre[7]), float(pre[8]), float(pre[9])
        out = []
        chunk = 1 << 20
        start = 1
        while start <= points:
            stop = min(points, start + chunk - 1)
            self.send(f":WAV:STAR {start}")
            self.send(f":WAV:STOP {stop}")
            out.extend(self.block(":WAV:DATA?"))
            start = stop + 1
        return xinc, xorig, [(b - yref - yorig) * yinc for b in out]


def active_span(volts, xinc, xorig, level, start_t):
    """start_t 以降、遷移が続いている長さ (us)。20 us 以上静かになったら終わりとみなす。"""
    quiet_limit = 20e-6
    last = None
    first = None
    for i, v in enumerate(volts):
        t = xorig + i * xinc
        if t < start_t or v <= level:
            continue
        if first is None:
            first = t
        if last is not None and t - last > quiet_limit:
            break
        last = t
    if first is None or last is None:
        return 0.0
    return (last - first) * 1e6


def first_crossing(volts, xinc, xorig, level, rising, after=None):
    """level を越える最初の位置を線形補間で返す。無ければ None。"""
    for i in range(1, len(volts)):
        t = xorig + i * xinc
        if after is not None and t < after:
            continue
        a, b = volts[i - 1], volts[i]
        if (rising and a < level <= b) or (not rising and a > level >= b):
            frac = (level - a) / (b - a) if b != a else 0.0
            return xorig + (i - 1 + frac) * xinc
    return None


def main() -> int:
    if not HOST:
        print("RIGOL_HOST is not set", file=sys.stderr)
        return 1
    shots = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    out_path = sys.argv[2] if len(sys.argv) > 2 else None

    s = Scope()
    print(s.query("*IDN?"))
    for ch in (1, 2):
        s.send(f":CHAN{ch}:DISP ON")
        s.send(f":CHAN{ch}:COUP DC")
        s.send(f":CHAN{ch}:PROB 1")
        s.send(f":CHAN{ch}:SCAL 1")
        s.send(f":CHAN{ch}:OFFS -1.5")
    s.send(":ACQ:MDEP 100k")
    s.send(f":TIM:SCAL {WINDOW_S / DIV:.9f}")
    # トリガを画面の左端へ。秒境界の直後だけを見たい。
    s.send(f":TIM:OFFS {WINDOW_S / 2:.9f}")
    s.send(":TRIG:MODE EDGE")
    s.send(":TRIG:EDGE:SOUR CHAN1")
    # 秒境界はアクティブ Low の立ち下がり。
    s.send(":TRIG:EDGE:SLOP NEG")
    s.send(":TRIG:EDGE:LEV 1.65")
    s.send(":TRIG:SWE SING")

    deltas = []
    spans = []
    rows = []
    for shot in range(shots):
        s.send(":SING")
        for _ in range(60):
            if s.query(":TRIG:STAT?") == "STOP":
                break
            time.sleep(0.1)
        else:
            print("  トリガがかからない", file=sys.stderr)
            continue

        xinc, xorig, ch1 = s.wave(1)
        _, _, ch2 = s.wave(2)
        if shot == 0:
            print(f"  分解能 {xinc * 1e9:.3f} ns/point、{len(ch1)} points、"
                  f"窓 {len(ch1) * xinc * 1e6:.0f} us")
        edge = first_crossing(ch1, xinc, xorig, 1.65, rising=False)
        burst = first_crossing(ch2, xinc, xorig, THRESH_V, rising=True, after=0.0)
        if edge is None or burst is None:
            print(f"  shot {shot}: エッジが見つからない (edge={edge}, burst={burst})")
            continue

        # 掴んだのがフレームかリンクパルスかを確かめる。フレームは 102 byte ぶん、10 Mbit/s で
        # 約 82 us にわたって遷移が続く。リンクパルスは 16 ms に 1 度の単発で、100 ns しかない。
        span = active_span(ch2, xinc, xorig, THRESH_V, burst)
        if span < 40.0:
            print(f"  shot {shot:3d}: 掴んだのは {span:.1f} us しか続かない — link pulse として捨てる")
            continue

        d = (burst - edge) * 1e6
        deltas.append(d)
        spans.append(span)
        rows.append(f"{shot} {edge * 1e6:.3f} {burst * 1e6:.3f} {d:.3f} {span:.3f}")
        print(f"  shot {shot:3d}: 秒境界から {d:8.2f} us  (活動 {span:6.1f} us)")

    if not deltas:
        print("測れた shot が無い", file=sys.stderr)
        return 1
    print()
    print(f"n={len(deltas)}  mean {statistics.mean(deltas):.2f} us  "
          f"median {statistics.median(deltas):.2f}  sd {statistics.pstdev(deltas):.2f}  "
          f"min {min(deltas):.2f}  max {max(deltas):.2f}")
    print(f"  掴んだ活動の長さ: median {statistics.median(spans):.1f} us "
          f"({min(spans):.1f}–{max(spans):.1f})。102 byte なら 81.6 us")
    if out_path:
        Path = __import__("pathlib").Path
        Path(out_path).write_text("\n".join(rows) + "\n", encoding="utf-8")
        print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
