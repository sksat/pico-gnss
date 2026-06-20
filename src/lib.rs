//! pico-gnss: RP2040 上で GNSS (秋月 AE-GNSS-EXTANT+ANT_SET) の NMEA を受信し、
//! PPS を観測するテストファーム。
//!
//! このクレートは lib + bin の 2 構成:
//! - lib (このファイル): HAL 非依存の純粋ロジック (GPSDO 規律・PPS 追跡・時刻同期・最小限の
//!   NMEA 抽出)。host で `cargo test-host` できる。
//! - bin ([src/main.rs](src/main.rs)): embassy-rp を使った firmware 本体。
//!
//! テスト時のみ std を有効化 (`cargo test-host`)。firmware ビルド時は no_std。
//!
//! # Features
//!
//! - **`nmea`** (default 無効): [`parse_rmc_time_date`] の RMC 解析を
//!   [`nmea`](https://docs.rs/nmea) crate に委譲する。default は依存ゼロの自前パーサ。
//!
//!   有効時の差 (実機 RP2040 / Cortex-M0+ @125MHz 実測, [src/bin/bench_nmea_parse.rs] 参照):
//!
//!   | | 自前 (default) | `nmea` |
//!   |---|---|---|
//!   | RMC 1 文の time+date 抽出 | **≈ 37 µs** | **≈ 619 µs** (≈ 17x 遅い) |
//!   | firmware `.text` 増分 | 0 (≈0.8KB の自前実体) | **+約 52KB** (nom/chrono 等 6 crate) |
//!   | checksum 検証 | なし | **あり** (不一致は `None`) |
//!   | 年の解釈 | 20xx 固定 | 世紀ピボット (`yy=94`→1994) |
//!   | 閏秒 `ss=60` | 受理 (次分へ繰上) | 拒否 (`None`) |
//!
//!   速度差は host では ≈4.3x だが M0+ は FPU 無しのため拡大する。いずれも 1Hz 用途では
//!   無視できる (nmea でも ≈0.06% CPU)。default (自前) を推奨し、checksum 検証や
//!   既存の `nmea` 依存を活かしたい場合のみ有効化する。
//! - **`bench`** (dev 専用): on-target ベンチ `bench_nmea_parse` バイナリを有効化する。
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
