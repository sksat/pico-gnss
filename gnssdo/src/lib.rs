//! `gnssdo`: GNSS PPS で規律されるクロック・holdover コア (GPS disciplined oscillator)。
//!
//! HAL 非依存・整数演算のみ・no_std・依存ゼロ (default) の純粋ロジック。MCU/ホストを問わず
//! **整数 ns タイムスタンプを渡す/受け取る**だけで動く (RP2040 で実動。STM32 等の入力キャプチャや
//! ホストの `/dev/pps` でも同様)。
//!
//! - [`DisciplinedClock`]: PPS 間隔から水晶の周波数オフセット (ppb) を EMA 推定し、PPS が
//!   切れている間 (holdover) も外挿して規律 UTC を保つ。capture/query の 2 timebase を扱う。
//! - [`PpsTracker`]: PPS エッジ列のロック/欠落/非単調を判定する。
//! - [`PpsTimeSync`]: NMEA 時刻と PPS エッジを対応付けて µs 精度の UTC エポックを確立する。
//! - NMEA 抽出ヘルパ ([`parse_rmc_time_date`] 等): time+date のみの最小実装 (full parse は不要)。
//!
//! このコアは NMEA 解析自体を要求しない ([`PpsTimeSync`] はパース済みの値を受け取る) ため、
//! 利用側は好きな NMEA パーサ (例 [`nmea`](https://docs.rs/nmea) crate) を併用できる。
//! RP2040 firmware は同 repo の `firmware/` クレート (embassy-rp) を参照。
//!
//! テスト時のみ std を有効化。通常は no_std。
//!
//! # Features
//!
//! - **`external-nmea`** (default 無効): [`parse_rmc_time_date`] の RMC 解析を
//!   [`nmea`](https://docs.rs/nmea) crate に委譲する (自前パースの代わり)。default は依存ゼロの自前パーサ。
//!
//!   有効時の差 (実機 RP2040 / Cortex-M0+ @125MHz 実測, firmware の `bench_nmea_parse` 参照):
//!
//!   | | 自前 (default) | `nmea` |
//!   |---|---|---|
//!   | RMC 1 文の time+date 抽出 | **≈ 37 µs** | **≈ 619 µs** (≈ 17x 遅い) |
//!   | 利用側 `.text` 増分 | 0 (≈0.8KB の自前実体) | **+約 52KB** (nom/chrono 等 6 crate) |
//!   | checksum 検証 | なし | **あり** (不一致は `None`) |
//!   | 年の解釈 | 20xx 固定 | 世紀ピボット (`yy=94`→1994) |
//!   | 閏秒 `ss=60` | 受理 (次分へ繰上) | 拒否 (`None`) |
//!
//!   速度差は host では ≈4.3x だが M0+ は FPU 無しのため拡大する。いずれも 1Hz 用途では
//!   無視できる (nmea でも ≈0.06% CPU)。default (自前) を推奨し、checksum 検証や
//!   既存の `nmea` 依存を活かしたい場合のみ有効化する。
#![cfg_attr(not(test), no_std)]

mod assembler;
mod gpsdo;
mod pps;
mod timesync;

pub use assembler::{NmeaLineAssembler, MAX_SENTENCE_LEN};
pub use gpsdo::{snap_to_second_ns, DisciplinedClock, DisciplinedClockConfig, FreqUpdate};
pub use pps::{PpsEvent, PpsTracker, PpsTrackerConfig, NOMINAL_US, TOLERANCE_US};
pub use timesync::{
    civil_to_unix, days_from_civil, parse_ddmmyy, parse_hhmmss, parse_rmc_time_date,
    PpsNmeaAssociation, PpsTimeSync, SyncPoint,
};
