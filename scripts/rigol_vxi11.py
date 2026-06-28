#!/usr/bin/env python3
"""VXI-11 transport for the Rigol DHO800, drop-in compatible with rigol_scpi.Rigol.

なぜ VXI-11 か: raw socket (5555) の SCPI セッションは、波形転送 (`:WAVeform:DATA?`) を途中で
中断すると「応答が 1 つ遅れる」状態に固着し、再接続でも *RST/*CLS でも抜けず power-cycle が要る
(実機で確認)。VXI-11 (port 111, ONC-RPC) は **proper request-response RPC framing** なので
off-by-one が原理的に起きず、さらに **device clear** でセッションをいつでもリセットできる。
よって観測系は VXI-11 を一次トランスポートにする (raw socket は fallback)。

依存: python-vxi11 (xdrlib を使うので **Python 3.12 以下**)。実行例:
  RIGOL_HOST=192.168.0.11 uv run --python 3.12 --with python-vxi11 python3 scripts/scope_logger.py
  (scope_logger は SCOPE_TRANSPORT=vxi11 で本クラスを使う)

インターフェース (rigol_scpi.Rigol と同じ): send/query/query_block/waveform/single/screenshot/
drain_errors/close と __enter__/__exit__。
"""
import os
import time

import vxi11


def _strip_block(raw: bytes) -> bytes:
    """IEEE-488.2 definite-length block #<n><len><payload> から payload を取り出す。"""
    i = raw.find(b"#")
    if i < 0:
        return raw
    ndig = int(raw[i + 1 : i + 2])
    start = i + 2 + ndig
    length = int(raw[i + 2 : start])
    return raw[start : start + length]


class RigolVxi11:
    """rigol_scpi.Rigol と同じ API の VXI-11 版。"""

    def __init__(self, host=None, timeout=6.0):
        host = host or os.environ.get("RIGOL_HOST")
        if not host:
            raise SystemExit("set RIGOL_HOST=<scope-ip>")
        self.dev = vxi11.Instrument(host)
        self.dev.timeout = timeout
        self.timeout = timeout

    def close(self):
        try:
            self.dev.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def clear(self):
        """VXI-11 device clear: SCPI 入出力バッファ/パーサをリセット (off-by-one 復旧)。"""
        self.dev.clear()

    def send(self, cmd):
        self.dev.write(cmd)

    def query(self, cmd):
        return self.dev.ask(cmd)

    def query_block(self, cmd):
        self.dev.write(cmd)
        return _strip_block(self.dev.read_raw())

    def drain_errors(self):
        out = []
        for _ in range(32):
            e = self.query(":SYSTem:ERRor?")
            if e.startswith("0,") or "No error" in e:
                break
            out.append(e)
        return out

    def waveform(self, ch, mode="NORMal"):
        self.dev.write(f":WAVeform:SOURce CHANnel{ch}")
        self.dev.write(f":WAVeform:MODE {mode}")
        self.dev.write(":WAVeform:FORMat BYTE")
        return self.query_block(":WAVeform:DATA?")

    def single(self, settle=0.05, tries=80):
        self.dev.write(":SINGle")
        for _ in range(tries):
            time.sleep(settle)
            if self.query(":TRIGger:STATus?") == "STOP":
                return True
        return False

    def screenshot(self, path):
        self.dev.write(":DISPlay:DATA? PNG")
        png = _strip_block(self.dev.read_raw())
        with open(path, "wb") as f:
            f.write(png)
        return len(png)


if __name__ == "__main__":
    # quick self-test: IDN + one waveform read over VXI-11.
    with RigolVxi11() as r:
        r.clear()
        print("IDN:", r.query("*IDN?"))
        print("errs:", r.drain_errors())
        print("sweep:", r.query(":TRIGger:SWEep?"))
        print("sdiv:", r.query(":TIMebase:MAIN:SCALe?"))
        w = r.waveform(1)
        print("CH1 waveform bytes:", len(w), "min/max:", min(w), max(w))
