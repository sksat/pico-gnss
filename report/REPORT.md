# pico-gnss 評価レポート — GPSDO 時刻同期・規律 PPS 出力

RP2040 (Raspberry Pi Pico) + 秋月 AE-GNSS-EXTANT (太陽誘電 GYSFFMANC / MediaTek MT3333) を
**窓際固定**で評価した結果。図は実機ログ ([sample-capture.log](sample-capture.log), 227s) から
`uv run webapp/plot_report.py` で生成。

![report](report.png)

## 到達点 (実機・窓際固定)

| 系統 | 指標 | 実測 | 対仕様 / 備考 |
|---|---|---|---|
| **PPS タイムスタンプ** | ジッタ σ | **12.8 ns** (p-p 64ns) | MT3333 1PPS 確度 **±10ns RMS** にほぼ一致 ✓ |
| **GPSDO 周波数** | 規律 freq / 安定度 | **+2.40 ppm / σ ~6.5 ppb** (定常) | RP2040 水晶を GPS に規律 |
| **時刻補正残差** | clock err σ | **9.4 ns** | ±10ns spec 帯内 ✓ (図④) |
| **holdover** | N秒断→誤差 | 26s → ~360ns / 2s → 43ns | 周波数外挿で sub-µs 保持 |
| **規律 PPS 出力 (PIO)** | 周期ジッタ | **16 ns** (PIO 1 tick, p-p 32ns) | ソフトタイミング比 ~1.5万倍クリーン |
| 規律 PPS 出力 | **UTC 位相同期** | **~±1.4 ms** (ソフト限界, 図①) | ns 位相は HW 必須 (後述) |
| 測位 | 水平 CEP | 12.5 m (この回) | 公称 2m。窓際の弱信号 (C/N₀≤29dBHz) が律速 |

host テスト 35 green。

## 各手法が効いている様子 (図の読み方)

横軸はすべて**起動からの実時間 [s]**、各パネルのタイトルがそのまま結論。

- **A. GPSDO ロック (log)**: `|freq − ロック値|` を log-y で。起動時 ~2400ppb のズレから **数秒で減衰**
  = 水晶ドリフトを学習しロック。以後は定常 (σ~数 ppb) で推移。log なので 3 桁の収束が 1 枚に乗る。
- **B. 時刻同期 ns 級**: GPSDO 補正後の UTC 予測残差。**MT3333 1PPS 仕様 ±10ns の帯 (緑) の内側**に
  収まる = ns 級の時刻同期。
- **C. 受信 PPS① と 自作規律 PPS② のジッタ**: ①GPS から受信した PPS入力、②自分で生成した規律PPS出力、
  を各平均からのズレで重ねた。**受信と同じ綺麗さ (PIO 16ns 量子化) で生成できている** (①は MT3333 仕様 ±10ns 相当)。
- **D. 位相同期の収束 (symlog)**: 規律出力エッジの UTC 秒境界からのズレ。起動から引き込むが、位相の
  「測定」が CPU の Instant 読み (executor ウェイクアップ遅延) 律速で **ソフトでは ±~1.4ms 止まり**。
  ns 位相同期は測定の HW 化 (PIO で出力エッジを GPS PPS と同じカウンタで捕捉, stage ②) が必要。

## 設計の核と教訓

1. **PIO ハードキャプチャ**: PPS を CPU 割込でなく PIO 自走カウンタで時刻印字 → M0+ の critical-section
   マスクに依存せず σ ~10ns (ソフトの ~9µs 下限を突破)。
2. **GPSDO**: PIO の精密 PPS 間隔で水晶 ppb を EMA 推定 → 時刻を規律。PPS 断中も周波数外挿 (holdover)。
3. **時刻補正の ns 化**: エポックも予測も PIO の ns 時刻にし、Instant の µs アンカーを回避 → err σ ~10ns。
4. **規律 PPS 出力**: SM1 が周期をハード生成、SM2 が GP3→GP4 ループバックで ns 計測。周波数=16ns。
5. **位相同期の限界 (重要な教訓)**: 制御対象 (周期) の精度 (ns) と、フィードバックの「測定」精度 (ms) は
   別物。位相を CPU の Instant で測ると executor のウェイクアップ遅延が乗り ms で頭打ち。**ns 位相同期は測定を
   ハード化 (PIO で出力エッジを GPS PPS と同じカウンタで捕捉) する stage ② が唯一の道**、と実測で確定。
   - 制御ループ自体の教訓: 1 サンプル遅れに連続比例制御をかけると `λ²−λ+k=0`。k=1 発振 / k=½ リンギング
     (周期8ステップ、実測と一致) / **k=¼ 臨界減衰**。詳細は [../NOTES.md](../NOTES.md) 罠12。

## 再現

```bash
# 配信と同時にログ記録
cd webapp && node dist/server.js --log /tmp/x.log
# テキスト集計 / 図生成 (uv が依存を隔離環境に自動構築)
uv run analyze.py /tmp/x.log
uv run plot_report.py /tmp/x.log out.png
```
