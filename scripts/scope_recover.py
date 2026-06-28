#!/usr/bin/env python3
"""Rigol DHO800 SCPI "delayed-by-one" の検出と (可能なら) ソフト復旧ツール。

  RIGOL_HOST=<scope-ip> python3 scripts/scope_recover.py

== 何が壊れているのか (the fault) ==

scope の SCPI セッションが **1 応答ずれ (off-by-one / delayed-by-one)** に陥ることがある。
すべての query が「**直前の** query の応答」を返す状態である。生 socket での実証:

  *IDN?                 を送ると "AUTO\\n" (直前の :TRIGger:SWEep? の答え) が返る
  :TIMebase:MAIN:SCALe? を送ると IDN 文字列が返る
  (send なしの純粋な recv は何も返らない — server は command 受信時だけ吐く)

これは TCP を張り直しても残り、*RST/*CLS や read-until-quiet ドレインでも消えなかった。
**根本原因**: zombie な scope_logger が ':WAVeform:DATA?' (バイナリブロック) の最中に
kill され、SCPI セッションが「ブロックを 1 個吐きかけ」の状態で取り残されたこと。
**ディスプレイ/取込は正常** (フリーズではない) — ずれているのは SCPI の query 整合だけ。

**scope は 1 クライアント限定**。このツール稼働中に別接続で scope を叩かないこと
(同時接続が SCPI セッションを再び壊す)。よって single connection / single client で動く。

== なぜ *IDN? 単独では検出できないか ==

off-by-one でも *IDN? は「直前の応答」を返すだけで、それがたまたま前の *IDN? なら
RIGOL を含んでしまう。**型の違う 3 クエリを連ねて、各応答が自分のクエリに合うか**を見る
必要がある (aligned())。整合していれば IDN→文字列 / SCALe→float / SWEep→AUTO|NORM|SING が
順番どおり返る。ずれていれば 2 番目以降が必ず型違反になる。

== 復旧メソッド (順に試し、各回 aligned() で確認、最初に整合したもので停止) ==

  (A) 完全な波形ブロック読み — 根本原因を直接狙う本命。
      中断した ':WAVeform:DATA?' が「ブロックを吐く途中」を残しているので、SOURce/MODE/FORMat
      を整えて query_block(':WAVeform:DATA?') を 1 回完走させ、IEEE-488.2 ブロックを丸ごと
      消費する。これでパーサが再同期しうる。
  (B) 無音までドレイン — *IDN? を 1 発送り、~3s 無音になるまで届くバイトを全部読み捨てる
      (滞留応答のバックログを空にする)。
  (C) パイプライン flush — *CLS / *IDN? / *IDN? を連続送信し、行応答を 3 本読み捨てる。
  (D) *CLS → *RST → 4s 待ち — *RST は設定をリセットする (後で scope_setup.py が再設定する)。

最後にもう一度 aligned() で判定し、'RECOVERED via <method>' (exit 0) か
'NOT recoverable in software — front-panel reset likely needed' (exit 1) を、観測した生応答を
証拠として添えて表示する。タイムアウトは crash させず「そのメソッドは失敗」として扱う。
"""
import os
import sys
import time

# scope_logger.py と同じ作法で skill 同梱の SCPI ヘルパを import する。
# cwd 非依存にするため __file__ 起点の絶対パスも通しておく (repo ルート相対も保険で残す)。
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_REPO, ".claude/skills/oscilloscope-timing/scripts"))
sys.path.insert(0, ".claude/skills/oscilloscope-timing/scripts")
from rigol_scpi import Rigol  # noqa: E402


def _is_float(s):
    """文字列が float としてパースできるか (例: '1.000000E-06')。"""
    try:
        float(s)
        return True
    except (ValueError, TypeError):
        return False


# 型の違う 3 クエリ。各 (command, その応答が自分のクエリに合うかの判定) のペア。
# off-by-one だと 2 番目以降が必ず型違反になるので、AND で確実に False になる。
_CHECKS = [
    ("*IDN?", lambda a: "RIGOL" in a.upper() or "DHO" in a.upper()),
    (":TIMebase:MAIN:SCALe?", _is_float),
    (":TRIGger:SWEep?", lambda a: any(t in a.upper() for t in ("AUTO", "NORM", "SING"))),
]


def _query_safe(rig, cmd):
    """query を投げ、タイムアウト等の例外は文字列化して返す (crash させない)。"""
    try:
        return rig.query(cmd)
    except Exception as e:  # noqa: BLE001 — socket.timeout 含め全部「応答なし」扱い
        return f"<error: {e!r}>"


def aligned(rig, evidence=None):
    """型の違う 3 クエリを発行し、各応答が自分のクエリに合うかを見て整合判定する。

    短絡せず常に 3 クエリ全部を発行する (off-by-one の連鎖状態を毎回同じにし、証拠も全部集めるため)。
    全部一致したときだけ True。`evidence` に dict を渡すと {command: 生応答} を書き込む。
    """
    results = []
    for cmd, check in _CHECKS:
        ans = _query_safe(rig, cmd)
        if evidence is not None:
            evidence[cmd] = ans
        results.append(check(ans))
    return all(results)


def _print_evidence(ev):
    """観測した生応答を証拠として表示する。"""
    for cmd, ans in ev.items():
        disp = ans if len(ans) <= 80 else ans[:77] + "..."
        print(f"     {cmd:>26} -> {disp!r}")


def method_a_waveform_block(rig):
    """(A) 完全な波形ブロック読み。中断した ':WAVeform:DATA?' の残りを 1 ブロック消費する。"""
    rig.send(":WAVeform:SOURce CHANnel1")
    rig.send(":WAVeform:MODE NORMal")
    rig.send(":WAVeform:FORMat BYTE")
    blk = rig.query_block(":WAVeform:DATA?")  # 例外は呼び出し側で捕捉
    return len(blk)


def method_b_drain_quiet(rig, quiet=3.0):
    """(B) *IDN? を 1 発送り、~quiet 秒 無音になるまで届くバイトを全部読み捨てる。"""
    rig.send("*IDN?")
    old = rig.s.gettimeout()
    rig.s.settimeout(quiet)
    last = time.time()
    try:
        while time.time() - last < quiet:
            try:
                d = rig.s.recv(4096)
                if not d:
                    break
                last = time.time()  # 何か来たら無音タイマをリセット
            except Exception:  # noqa: BLE001 — recv timeout = 無音到達
                break
    finally:
        rig.s.settimeout(old)


def method_c_pipelined_flush(rig):
    """(C) *CLS / *IDN? / *IDN? を連続送信し、行応答を 3 本読み捨てる。"""
    rig.send("*CLS")
    rig.send("*IDN?")
    rig.send("*IDN?")
    old = rig.s.gettimeout()
    rig.s.settimeout(rig.timeout)
    try:
        for _ in range(3):
            data = b""
            try:
                while not data.endswith(b"\n"):
                    chunk = rig.s.recv(4096)
                    if not chunk:
                        break
                    data += chunk
            except Exception:  # noqa: BLE001 — 応答が尽きたら timeout する。捨てて次へ
                break
    finally:
        rig.s.settimeout(old)


def method_d_reset(rig):
    """(D) *CLS → *RST → 4s 待ち。*RST は設定をリセットする (後で scope_setup.py が再設定)。"""
    rig.send("*CLS")
    rig.send("*RST")
    time.sleep(4.0)


_METHODS = [
    ("A: full waveform-block read", method_a_waveform_block),
    ("B: drain-until-quiet", method_b_drain_quiet),
    ("C: pipelined flush", method_c_pipelined_flush),
    ("D: *CLS;*RST;wait 4s", method_d_reset),
]


def main():
    if not os.environ.get("RIGOL_HOST"):
        print("set RIGOL_HOST=<scope-ip> (do not hardcode the IP)", file=sys.stderr)
        return 2

    try:
        rig = Rigol(timeout=4.0)
    except Exception as e:  # noqa: BLE001
        print(f"connect failed (is the scope reachable? is another client connected?): {e!r}",
              file=sys.stderr)
        return 2

    try:
        # まず現状を判定。
        ev = {}
        if aligned(rig, ev):
            print("already aligned")
            _print_evidence(ev)
            return 0

        print("misaligned: SCPI delayed-by-one detected")
        _print_evidence(ev)

        # メソッドを順に試し、各回 aligned() で確認。最初に整合したもので停止。
        for name, fn in _METHODS:
            print(f"-- trying method {name} ...")
            try:
                fn(rig)
            except Exception as e:  # noqa: BLE001 — timeout 等は「このメソッドは失敗」扱い
                print(f"   method raised (treated as failure): {e!r}")
            ev = {}
            try:
                ok = aligned(rig, ev)
            except Exception as e:  # noqa: BLE001
                ok = False
                print(f"   alignment check raised: {e!r}")
            _print_evidence(ev)
            if ok:
                print(f"RECOVERED via {name}")
                return 0

        # 全メソッド後、最終判定をもう一度取る。
        ev = {}
        try:
            ok = aligned(rig, ev)
        except Exception as e:  # noqa: BLE001
            ok = False
            print(f"final alignment check raised: {e!r}")
        _print_evidence(ev)
        if ok:
            print("RECOVERED (aligned on final check)")
            return 0

        print("NOT recoverable in software — front-panel reset likely needed")
        return 1
    finally:
        rig.close()


if __name__ == "__main__":
    sys.exit(main())
