# pico-gnss webapp

`pico-gnss` firmware が defmt-rtt (probe-rs) に流す GNSS データをリアルタイム可視化する
ダッシュボード。地図・スカイプロット・C/N0 (SNR)・PPS・PPS 規律 UTC・生 NMEA を表示する。

- **データ経路**: probe-rs の RTT 出力 (または録画 replay) → Node ブリッジ (`src/server.ts`) が
  `NMEA` / `PPS` / `SYNC` 行を抽出 → WebSocket でブラウザへ。NMEA のパースはブラウザ (`src/client/client.ts`)。
- **依存**: ランタイムは Node 組み込みのみ (WebSocket も最小自前実装、ゼロランタイム依存)。
  ビルドに TypeScript、地図に Leaflet (CDN)。

## 使い方

```sh
pnpm install
pnpm build

# A) 録画 replay (ハード不要。sample.log を再生)
pnpm replay

# B) 実機ライブ (PicoBridge Lite 接続。probe-rs run でフラッシュ+配信)
pnpm start
#    既にフラッシュ済みなら attach で:  node dist/server.js --attach
```

ブラウザで <http://localhost:8080> を開く。

### オプション (`node dist/server.js ...`)
- `--replay <file>` 録画ファイルを秒ペースで再生 (ループ)
- `--attach` `probe-rs run` の代わりに `attach` (再フラッシュしない)
- `--elf <path>` firmware ELF (既定 `../target/thumbv6m-none-eabi/debug/pico-gnss`)
- `--chip <chip>` 既定 `RP2040` ・ `--port <n>` 既定 `8080`

## firmware が出す行 (server がパース)
```
NMEA $GxXXX,...*hh
PPS count=<n> interval_us=<us> state=<First|Locked|Irregular> missed=<m>
SYNC pps_local_us=<t> unix_s=<s> drift_us=<d>
```
`SYNC` は PPS エッジ↔UTC 秒の対応付けを firmware (RP2040, 1µs) 側で行った結果
(精度のため host では同期しない)。`sample.log` は実測 fix の録画。
