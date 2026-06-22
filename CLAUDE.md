# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

`pico-gnss-rs` は RP2040 で GNSS の PPS 時刻同期を扱うプロジェクトである。
GNSS の 1PPS を PIO で ns 分解能で捕捉し、水晶を規律して規律 UTC と規律 1PPS 出力を保ち (GPSDO。PPS が切れているあいだも holdover で外挿する)、PPS を NMEA の UTC 秒に対応づけ、リアルタイムに可視化する。
規律コアは HAL 非依存の再利用可能な `no_std` crate に切り出してある。

## 開発の作法

### 0. 開発は TDD で行う

コア (`gnssdo` と `rp-pps` の HAL 非依存部) は no_std、整数のみ、HAL 非依存で書いてあり、host で `cargo test` できる。
期待する動作を fail するテストとして先に書き、それを pass させ、構造を refactor する。
周波数推定、位相 servo、NMEA、PPS と UTC 秒のペアリング、PIO の tick と ns の換算といったロジックは host テストで覆う。
物理レンジの proptest と、`#[ignore]` のモデル検証や実ログ replay も使う (例: `gnssdo/tests/thermal_plant.rs`)。
PIO の実挙動、受信、温度といった HW 依存は実機検証になるが、これは TDD の代わりではなく、**TDD で覆えない最後の層**として扱う。

### 1. 設計はまず smart-friend に相談する

新しい設計、API 境界、テスト戦略、制御則は、実装前に smart-friend で外部 AI の別視点を取る。
実装後のコードレビューには code-review-gpt を使う。
どちらも関連ファイルを読ませ、観点を明示して呼ぶ。
回答は鵜呑みにせず、自分の見解も持った上で扱う。

### 2. cargo test と実機を信じ、直感を信じない

「コードを読んで大丈夫そう」「テストを直したから繋がっているはず」では進めない。
host は `cargo test`、HW は probe-rs の生出力やオシロの密な計測が真である。
失敗したり空だったりするツール出力を事実として扱わない。
制御位相や受信のように揺れる量は、単発のスナップショットや緩く時刻を合わせただけの比較で判断しない。
密な多 shot と統計 (平均、標準偏差、相関) で見て、failure を新しい計測で**再現**するまで「直した」と言わない。
host のプラントモデルが示す余裕 (ゲインの headroom など) も採用の根拠にはしない。
採用の可否は実機の指標 (ロック保持など) が決め、モデルは欠けている制約の方向に必ず楽観するからである。

## レイヤ構成

2 層のライブラリと firmware の 3 段からなる。
**層の境界を跨ぐのは整数ナノ秒のタイムスタンプと UTC エポックだけで、HAL の型は跨がない。**
これがコアを host や任意の MCU で動かせてテストもできる理由なので、境界に HAL の型や浮動小数を持ち込まない。

- **`gnssdo`**: 移植可能な規律コア。水晶の周波数オフセット (ppb) を推定して holdover する `DisciplinedClock`、PPS エッジを分類する `PpsTracker`、出力位相の type-II servo `PhaseLockLoop` (P/I/D と Smith 予測子)、受信機の量子化誤差を引く `QErrCorrector` を持つ。no_std、整数のみ、依存ゼロ。
- **`rp-pps`**: その入力を作る RP2040/RP2350 の PIO building block。PIO で 1PPS を ~16ns でハード捕捉し (GPIO 割込の µs ジッタを避ける)、操舵できる 1PPS を出し、NMEA を framing と parse し、PPS と UTC 秒をペアリングする。HAL 非依存の core (host テスト可) に、薄い embassy-rp / rp2040-hal backend を載せた構成。
- **`pico-gnss`**: 上記を embassy で配線する実機ファーム。

`webapp/` は別系統の Node アプリで、probe-rs の RTT (defmt) 出力か replay ファイルから firmware の行を取り、リアルタイムダッシュボードで配信する (座標を隠す privacy mode を持つ)。

## ビルド、テスト、注意点

- 素の `cargo test` と `cargo build` は host で動くコア (`default-members = gnssdo, rp-pps`) だけを対象にする。単一テストは `cargo test -p gnssdo <name>`。
- firmware は `cd pico-gnss && cargo build --release` で建てる (target `thumbv6m-none-eabi` と linker は `.cargo/config` に設定済み)。
- 実機フラッシュは `cargo build --release && cargo run --release` とする。`;` でなく `&&` にするのは、ビルド失敗時に古いバイナリを焼かないため。runner は probe-rs で、ログは `> logs/pps-<date>.log` に残す。
- `#[ignore]` のモデル検証、実ログ replay、掃引テストは手動で走らせる。`cargo test -p gnssdo --test thermal_plant -- --ignored <name> --nocapture` のように呼び、replay は `GNSSDO_REPLAY_LOG=` などの env を渡す。
- RTT に何も出ないときは `DEFMT_LOG` を疑う。未設定だと `info!` や `warn!` がコンパイル時に消えるためで、`.cargo/config` で `info` に設定済み。
- 実験ログやその解析結果に出る位置座標は、**commit せず、使う前にマスクする**。生の RTT ログ (`logs/*.log`) は gitignore 済みで、`report/` にはマスク済みの結論や図だけを置く。
- 機器の所在を直書きしない。オシロや受信機の IP は env (`RIGOL_HOST` など) で渡す。
- オシロでのタイミングと位相の計測は、レシピと落とし穴を oscilloscope-timing skill にまとめてある (新しい罠を踏んだら追記しておくとよい)。
