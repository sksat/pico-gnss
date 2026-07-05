#!/usr/bin/env python3
"""Raw socket (5555) transport for the Rigol DHO800, drop-in compatible with
rigol_vxi11.RigolVxi11 / scope_pps.Rigol。VXI-11 の read が死んだ session 用のフォールバック。

DHO804 の 5555 は健全時は普通の request-response だが、固着すると **lazy-flush** になる:
コマンド受信を契機に「それ以前に pending だった応答」を push し、そのコマンド自身の応答は
次のコマンドまで pending に留まる。さらに壊れた engine は直前応答の複製を散発的に差し込む。
本クラスはどちらのモードでも同じコードで動くよう、text query を **マーカー同期**にする:

    drain; send(cmd); send(*IDN?)   # 実クエリ *IDN? が pusher (lazy でも cmd 応答を押し出す)
    → 受信行のうち IDN 行 ('RIGOL TECHNOLOGIES,...') 以外の最後の非空行が cmd の応答

pusher に *CLS を使わないのは、*CLS が「出力キューのクリア」でもあり、cmd の応答生成前に
処理されると応答ごと消すレースがあるため。pusher *IDN? 自身の応答は lazy では pending に
残るが、次のコマンド送信で押し出されて IDN 行として読み捨てられるので自己整合する。
binary block (:WAV:DATA?, :DISP:DATA?) は実クエリ (:WAV:FORMat?) を pusher にして
skip-to-'#' で読む。:SINGle 後は status polling せず固定 wait (>1 PPS 周期) で
1 フレーム取る (polling が捕捉フレームを消すため)。

実行例 (checkpoint/longrun 系は SCOPE_TRANSPORT=raw で本クラスに切り替わる):
  RIGOL_HOST=192.168.0.11 SCOPE_TRANSPORT=raw python3 logs/.../longrun.py abab 3600
"""
import os
import socket
import time

IDN_MARK = b"RIGOL TECHNOLOGIES"


class RigolRaw:
    """rigol_vxi11.RigolVxi11 と同じ API の raw socket 版。"""

    def __init__(self, host=None, port=None, timeout=6.0):
        host = host or os.environ.get("RIGOL_HOST")
        if not host:
            raise SystemExit("set RIGOL_HOST=<scope-ip>")
        port = port or int(os.environ.get("RIGOL_PORT", "5555"))
        self.s = socket.create_connection((host, port), timeout=timeout)
        self.timeout = timeout

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def send(self, cmd):
        self.s.sendall((cmd + "\n").encode())

    def _drain(self, t=0.3):
        """socket に届いているバイトを読み捨てる。"""
        self.s.settimeout(t)
        got = b""
        try:
            while True:
                d = self.s.recv(65536)
                if not d:
                    break
                got += d
        except (socket.timeout, OSError):
            pass
        return got

    def clear(self):
        """*CLS + drain を空になるまで繰り返して backlog を掃除する。"""
        for _ in range(12):
            self.send("*CLS")
            if not self._drain(0.3):
                break

    def query(self, cmd):
        """マーカー同期 text query。stale/複製応答を読み捨てて cmd の応答だけ返す。"""
        self._drain(0.05)  # 前回の pusher 応答などの残りを捨てる
        self.send(cmd)
        self.send("*IDN?")  # 実クエリ pusher
        want_idn = cmd.strip().upper().startswith("*IDN")
        deadline = time.monotonic() + self.timeout
        buf = b""
        candidate = None
        while time.monotonic() < deadline:
            # 候補が出たら短い grace だけ待ち、健全モードなら pusher の IDN 行で即確定
            self.s.settimeout(0.4 if candidate else max(0.1, deadline - time.monotonic()))
            try:
                d = self.s.recv(4096)
            except (socket.timeout, OSError):
                if candidate is not None:
                    break
                continue
            if not d:
                break
            buf += d
            lines = buf.split(b"\n")[:-1]  # 完結した行だけ見る
            idn_after_candidate = False
            candidate = None
            for line in lines:
                t = line.strip()
                if not t:
                    continue
                if IDN_MARK in t:
                    if want_idn:
                        candidate = t
                    elif candidate is not None:
                        idn_after_candidate = True
                else:
                    candidate = t
            if idn_after_candidate or (want_idn and candidate):
                break
        if candidate is None:
            raise TimeoutError(f"query {cmd!r}: no marker-synced response")
        return candidate.decode(errors="replace").strip()

    def drain_errors(self):
        out = []
        for _ in range(8):
            e = self.query(":SYSTem:ERRor?")
            if e.startswith("0,") or "No error" in e:
                break
            out.append(e)
        return out

    def _read_exact(self, n, timeout=8.0):
        buf = b""
        self.s.settimeout(timeout)
        while len(buf) < n:
            d = self.s.recv(n - len(buf))
            if not d:
                raise EOFError("connection closed mid-block")
            buf += d
        return buf

    def _read_block(self):
        """#<n><len><payload> を skip-to-'#' で読む (直前に実クエリ pusher を送ってあること)。"""
        for _ in range(4096):
            if self._read_exact(1) == b"#":
                break
        else:
            raise RuntimeError("no block header")
        ndig = int(self._read_exact(1))
        ln = int(self._read_exact(ndig))
        payload = self._read_exact(ln)
        self._drain(0.2)  # 末尾改行と pusher (:WAV:FORMat?) の応答を捨てる
        return payload

    def query_block(self, cmd):
        self._drain(0.1)
        self.send(cmd)
        self.send(":WAVeform:FORMat?")  # 実クエリ pusher: block を abort せず flush する
        return self._read_block()

    def waveform(self, ch, mode="NORMal"):
        self.send(f":WAVeform:SOURce CHANnel{ch}")
        self.send(f":WAVeform:MODE {mode}")
        self.send(":WAVeform:FORMat BYTE")
        return self.query_block(":WAVeform:DATA?")

    def single(self, settle=None, tries=None):
        """:SINGle + 固定 wait。status polling は *CLS が捕捉フレームを消すのでしない。
        settle/tries は VXI-11 版との signature 互換のためだけにあり、無視する。"""
        self.send(":SINGle")
        time.sleep(1.35)  # >1 PPS 周期: 独立な triggered フレームを 1 枚確定させる
        self._drain(0.1)
        return True

    def screenshot(self, path):
        png = self.query_block(":DISPlay:DATA? PNG")
        with open(path, "wb") as f:
            f.write(png)
        return len(png)


if __name__ == "__main__":
    # quick self-test: IDN + waveform + screenshot over raw 5555.
    with RigolRaw() as r:
        r.clear()
        print("IDN:", r.query("*IDN?"))
        print("errs:", r.drain_errors())
        print("sdiv:", r.query(":TIMebase:MAIN:SCALe?"))
        w = r.waveform(1)
        print("CH1 waveform bytes:", len(w), "min/max:", min(w), max(w))
        n = r.screenshot("/tmp/rigol_raw_selftest.png")
        print("screenshot bytes:", n)
