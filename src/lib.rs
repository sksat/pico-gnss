//! pico-gnss: RP2040 上で GNSS (秋月 AE-GNSS-EXTANT+ANT_SET) の NMEA を受信し、
//! PPS を観測するテストファーム。
//!
//! このクレートは lib + bin の 2 構成:
//! - lib (このファイル): HAL 非依存の純粋ロジック。host で `cargo test-host` できる。
//!   既存ライブラリが埋めない部分だけを置く — NMEA パース自体は firmware 側で
//!   `nmea` クレートに委譲し、ここには重複させない。
//! - bin ([src/main.rs](src/main.rs)): embassy-rp を使った firmware 本体。
//!
//! テスト時のみ std を有効化 (`cargo test-host`)。firmware ビルド時は no_std。
#![cfg_attr(not(test), no_std)]

mod assembler;
mod gpsdo;
mod pps;
mod timesync;

pub use assembler::{NmeaLineAssembler, MAX_SENTENCE_LEN};
pub use gpsdo::{snap_to_second_ns, DisciplinedClock, FreqUpdate};
pub use pps::{PpsEvent, PpsTracker, NOMINAL_US, TOLERANCE_US};
pub use timesync::{
    civil_to_unix, days_from_civil, parse_ddmmyy, parse_hhmmss, PpsTimeSync, SyncPoint,
};
