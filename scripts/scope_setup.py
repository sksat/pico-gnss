#!/usr/bin/env python3
"""Rigol DHO800 を 2 チャネルの立ち上がりエッジ・タイミング計測向けに一発設定する (send-only)。

  RIGOL_HOST=<scope-ip> python3 scripts/scope_setup.py

**なぜ send-only か**: scope の SCPI セッションは、別クライアントが binary block
(`:WAVeform:DATA?`) の途中で殺されると「1 応答ずれ」(各 query が直前の query の応答を返す)
に陥ることがある。**送信だけのコマンドはこのずれの影響を受けない** (応答を読まないため) ので、
設定は全て `.send()` で投げる。query を使う realign は別途 scope_logger 側の `_realign` が行う。

設定内容 (このベンチの物理結線に固定):
  CH1 = GPS PPS     : probe 1x (同軸直結), 1 V/div, offset -1.5V, DISPlay ON
  CH2 = GPSDO 出力     : probe 10x (×10 プローブ), 1 V/div, offset -1.5V, DISPlay ON
  Trigger           : EDGE, source CH1, slope POSitive, level 1.65V, sweep NORMal
  Timebase          : 200 ns/div (2e-7), offset 0
  最後に            : :MEASure:CLEar, :RUN

oscilloscope-timing SKILL.md の落とし穴を踏まえた要点:
  - #8 probe ratio は物理結線に合わせる。CH1 は 1x 直結なのに 10x を押し込むと 3.3V×10=33V で
    画面上端を突き抜け、生バイトが railed になり絶対電圧が偽る。CH1=1x / CH2=10x で固定する。
  - #3 トリガレベルは probe 減衰の **後** で効く。先に PROBe を実比に設定してから LEVel を決める
    ので、本スクリプトは各 CH の PROBe を SCALe/OFFSet/LEVel より先に送る。
  - #7 1PPS のような 1Hz の低レート信号は sweep を必ず NORMal にする。AUTO だとエッジを待たず
    自動トリガして未同期波形を出し続ける。
  - #6 :MEASure:CLEar は引数を取らない (ALL を付けると -108)。
  - #7 垂直 offset を誤ると 3.3V が clip して「信号なし」と誤判定する。1 V/div + offset -1.5V で
    0–3.3V を画面内に収める。

接続先 IP は RIGOL_HOST env から取る (機器の所在は直書きしない)。実行は repo ルートから。
"""
import os
import sys
import time

sys.path.insert(0, ".claude/skills/oscilloscope-timing/scripts")
from rigol_scpi import Rigol

REF_CH = int(os.environ.get("REF_CH", "1"))  # GPS PPS (trigger 基準), 1x 直結
SIG_CH = int(os.environ.get("SIG_CH", "2"))  # GPSDO 出力, 10x プローブ

# 送るコマンド列。順序が重要: 各 CH は PROBe を SCALe/OFFSet より先に
# (probe 比が縦軸/レベルの解釈を決める。SKILL #3)。
SETUP_COMMANDS = [
    # --- CH1 = GPS PPS, 1x 直結 ---
    f":CHANnel{REF_CH}:DISPlay ON",
    f":CHANnel{REF_CH}:PROBe 1",        # 物理結線に合わせる (1x 直結)。SKILL #8
    f":CHANnel{REF_CH}:COUPling DC",    # ロジックレベルのエッジは DC 結合で
    f":CHANnel{REF_CH}:SCALe 1",        # 1 V/div
    f":CHANnel{REF_CH}:OFFSet -1.5",    # 0–3.3V を画面内に収める。SKILL #7
    # --- CH2 = GPSDO 出力, 10x プローブ ---
    f":CHANnel{SIG_CH}:DISPlay ON",
    f":CHANnel{SIG_CH}:PROBe 10",       # ×10 プローブに合わせる。SKILL #8
    f":CHANnel{SIG_CH}:COUPling DC",
    f":CHANnel{SIG_CH}:SCALe 1",        # 1 V/div
    f":CHANnel{SIG_CH}:OFFSet -1.5",
    # --- Trigger: CH1 立ち上がりエッジ, 1.65V, NORMal ---
    ":TRIGger:MODE EDGE",
    f":TRIGger:EDGE:SOURce CHANnel{REF_CH}",
    ":TRIGger:EDGE:SLOPe POSitive",
    ":TRIGger:EDGE:LEVel 1.65",         # 画面に出る電圧で決める (PROBe 設定後)。SKILL #3
    ":TRIGger:SWEep NORMal",            # 1Hz の 1PPS は NORMal 必須。SKILL #7
    # --- Timebase: 200 ns/div, offset 0 ---
    ":TIMebase:MAIN:SCALe 2e-7",        # 200 ns/div
    ":TIMebase:MAIN:OFFSet 0",
    # --- 計測ウィンドウを消して波形ビューを確保 → 連続取得開始 ---
    ":MEASure:CLEar",                   # 引数を取らない。SKILL #6
    ":RUN",
]

# best-effort の検証 query。off-by-one (1 応答ずれ) が残っていると応答が直前のコマンドの
# ものになるが、ここでは失敗させず「届いた生応答」をそのまま出すだけ (期待値も併記)。
VERIFY_QUERIES = [
    (f":CHANnel{REF_CH}:PROBe?", "1"),
    (f":CHANnel{REF_CH}:SCALe?", "1 (V/div)"),
    (f":CHANnel{REF_CH}:OFFSet?", "-1.5"),
    (f":CHANnel{REF_CH}:DISPlay?", "1/ON"),
    (f":CHANnel{SIG_CH}:PROBe?", "10"),
    (f":CHANnel{SIG_CH}:SCALe?", "1 (V/div)"),
    (f":CHANnel{SIG_CH}:OFFSet?", "-1.5"),
    (f":CHANnel{SIG_CH}:DISPlay?", "1/ON"),
    (":TRIGger:MODE?", "EDGE"),
    (":TRIGger:EDGE:SOURce?", f"CHAN{REF_CH}"),
    (":TRIGger:EDGE:SLOPe?", "POS"),
    (":TRIGger:EDGE:LEVel?", "1.65"),
    (":TRIGger:SWEep?", "NORM"),
    (":TIMebase:MAIN:SCALe?", "2e-7"),
    (":TIMebase:MAIN:OFFSet?", "0"),
]


def main():
    scope = Rigol(timeout=4.0)
    try:
        print(f"configuring scope at {os.environ.get('RIGOL_HOST')} "
              f"(REF=CH{REF_CH} GPS/1x, SIG=CH{SIG_CH} out/10x)")
        for cmd in SETUP_COMMANDS:
            scope.send(cmd)
            print(f"  send: {cmd}")
            time.sleep(0.05)  # 各コマンドに少しだけ整定時間を与える
        print("setup commands sent (send-only; off-by-one read fault does not affect these).")

        # best-effort verify: query は off-by-one の可能性があるので失敗させない。
        print("\nbest-effort verify (off-by-one の可能性あり; 生応答をそのまま表示):")
        for cmd, expect in VERIFY_QUERIES:
            try:
                got = scope.query(cmd)
            except Exception as e:
                got = f"<query failed: {e}>"
            print(f"  {cmd:32s} -> {got!r}   (expect ~{expect})")
        print("\nNOTE: 応答が 1 つずれている/期待値と食い違うなら read off-by-one が残存。"
              "設定自体は send 済みなので有効。読みの整合は scope_logger の _realign で取る。")
    finally:
        scope.close()


if __name__ == "__main__":
    main()
