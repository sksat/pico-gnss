# pico-gnss webapp

`pico-gnss` firmware が defmt-rtt (probe-rs) に流す GNSS データをリアルタイム可視化する
ダッシュボード (**React 19 + Vite + TypeScript**)。

## 構成
- **ブリッジ `src/server.ts`** (Node, ランタイム依存ゼロ): probe-rs の RTT 出力 (または録画
  replay) から `NMEA` / `PPS` / `SYNC` 行を抽出し、WebSocket で配信。Vite ビルド成果物
  (`public/`) も同一ポートで静的配信。WebSocket もサーバ→クライアント送出だけ最小自前実装。
- **フロント `web/`** (React + Vite): `web/src/nmea.ts` で NMEA をパース (QZSS/SBAS は PRN
  レンジ判定)、`web/src/components/` の各パネルを描画。地図は react-leaflet (CARTO Voyager)。

## 可視化
fix サマリ / 地図 (現在地+軌跡) / スカイプロット / C/N₀ バー / **衛星テーブル** /
**測位精度** (DOP・2DRMS/CEP・散布図) / **PPS 時刻精度** (jitter σ・ヒストグラム・ppm) /
**時系列** (高度・衛星数・速度) / PPS規律 UTC / 生 NMEA コンソール (ドラッグでリサイズ)。

## 使い方

```sh
pnpm install
pnpm build          # tsc (server) + vite build (web) → public/

# A) 録画 replay (ハード不要。sample.log を再生)
pnpm replay

# B) 実機ライブ (PicoBridge Lite 接続。probe-rs run でフラッシュ+配信)
pnpm start
#    既にフラッシュ済みなら:  node dist/server.js --attach
```

ブラウザで <http://localhost:8137> を開く。

### server オプション (`node dist/server.js ...`)
`--replay <file>` 録画再生 (ループ) ・ `--attach` 再フラッシュせず attach ・
`--elf <path>` (既定 `../target/thumbv6m-none-eabi/debug/pico-gnss`) ・
`--chip <chip>` (既定 RP2040) ・ `--port <n>` (既定 8137) ・
`--log <file>` 配信と同時に生ログを記録 (ダッシュボードを見ながらキャプチャ)。

### オフライン解析 `analyze.py`
`--log` で録ったログ (または `probe-rs run > x.log`) を集計するダッシュボードのオフライン版:
```bash
python3 analyze.py /tmp/eval.log
```
測位精度 (CEP/R95/2DRMS)・PPS ジッタ・GPSDO 安定度・時刻補正残差 (snap 済)・規律 PPS 出力ジッタ・
holdover 誤差・衛星/C/N0 を出す。±1ms フィルタ・snap は firmware と揃えてある。

## firmware が出す行 (server がパース)
```
NMEA $GxXXX,...*hh
PPS count=<n> interval_us=<us> state=<First|Locked|Irregular> missed=<m>
SYNC pps_local_us=<t> unix_s=<s> drift_us=<d>
```
`SYNC` は PPS エッジ↔UTC 秒の対応付けを firmware (RP2040, 1µs) 側で行った結果。
`sample.log` は実測 fix の録画 (replay 用)。
