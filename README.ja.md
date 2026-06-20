# pico-gnss-rs

[![crates.io](https://img.shields.io/crates/v/gnssdo.svg)](https://crates.io/crates/gnssdo)
[![docs.rs](https://img.shields.io/docsrs/gnssdo)](https://docs.rs/gnssdo)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

[English](README.md) | **日本語**

RP2040 上に構築した GNSS **PPS 規律クロック (GPSDO/GNSSDO)**。再利用可能な `no_std`
コアクレートと、リアルタイム Web ダッシュボードを含みます。

GNSS 受信機の 1Hz PPS 立ち上がりは UTC 秒境界を刻みます。本プロジェクトはそのエッジを
ナノ秒分解能で捕捉し、ローカル水晶の周波数誤差を推定して、**PPS が切れている間
(holdover) も含めて**規律された UTC を保ちます。

## ワークスペース構成

| パス | 内容 |
|---|---|
| [`gnssdo/`](gnssdo/) | **規律コア** ([crate `gnssdo`](gnssdo/README.md))。`no_std`・HAL 非依存・整数演算のみ・**依存ゼロ**。周波数 (ppb) 推定、holdover、PPS エッジ追跡、出力位相ロック servo (PLL)。timestamp と UTC epoch を消費。host テスト可能。 |
| [`rp-pps/`](rp-pps/) | **RP2040/RP2350 PIO + 受信機 I/O** (crate `rp-pps`)。PPS エッジのハード捕捉・操舵可能な 1PPS 出力、NMEA フレーミング/パース、PPS↔UTC 秒の対応付け — `gnssdo` が消費する timestamp と epoch を生産。HAL 非依存コア (host テスト可) + embassy-rp / rp2040-hal backend。 |
| [`pico-gnss/`](pico-gnss/) | RP2040 firmware (embassy-rp)。PIO ハード PPS 捕捉、クロック規律、規律 PPS 出力。embedded 専用で `gnssdo` + `rp-pps` を配線。 |
| [`webapp/`](webapp/) | リアルタイムダッシュボード (React 19 + Vite + TypeScript)。firmware の defmt/RTT 出力を依存ゼロの Node ブリッジ経由で表示。 |
| [`report/`](report/) | 実機評価のログと図。 |
| [`NOTES.md`](NOTES.md) | 設計判断とハマった罠の記録。 |

## クイックスタート

```sh
# コアライブラリ — ハード不要で host 上で動く:
cargo test -p gnssdo

# firmware — probe-rs 対応プローブ + RP2040 が必要:
cd pico-gnss && cargo run --release       # build → flash → defmt ログを表示
```

ワークスペースの `default-members` は `gnssdo` なので、ルートでの素の
`cargo build`/`cargo test` は host 安全なコアだけを対象にします。firmware は
embedded 専用で、`pico-gnss/`(その `.cargo/config.toml` が `thumbv6m-none-eabi`
ターゲットと probe-rs runner を選ぶ)の中からビルドします。

## ハードウェア

- **MCU**: RP2040 (Raspberry Pi Pico, Seeed XIAO RP2040 など)。
- **GNSS モジュール** (NMEA + 1PPS 出力)。例: 秋月 `AE-GNSS-EXTANT` /
  GYSFFMANC (MediaTek MT3333)、9600 baud。
- **配線**: UART0 RX = GP1 (モジュール TX)、UART0 TX = GP0 (モジュール RX)、PPS = GP2。
  共通 GND が必要。

## 実機評価結果 (RP2040 @ 125 MHz)

![evaluation report](report/report.png)

実機ログ ([`sample-capture.log`](report/sample-capture.log), ~227s) から
`uv run webapp/plot_report.py` で生成:

- **A** — GPSDO が起動時に水晶ドリフト (~+2.5 ppm) を学習し、その後 ppb 級で保持する。
  これが holdover を可能にする。
- **B** — 時刻補正残差 σ ≈ 数十 ns で、**受信機の ±10 ns 1PPS 仕様の内側**。
- **C** — PPS ジッタは 16 ns 捕捉量子化の数段階に収まる (PIO ハード捕捉、~10–16 ns。
  ソフト GPIO 割込方式の ~9 µs に対して)。
- **D** — 規律 PPS 出力は UTC 秒境界に **σ ~35–48 ns** で位相ロック (Smith 予測子サーボ。
  旧ソフトサーボは ±1.4 ms)。

before/after — 位相の「測定」精度 (Instant ±ms → PIO 16 ns) と、その結果の出力位相:

![before/after](report/compare.png)

手法の詳細・全図は [`report/REPORT.md`](report/REPORT.md) を参照。
