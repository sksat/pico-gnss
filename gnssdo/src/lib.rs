//! `gnssdo`: a GNSS-PPS-disciplined clock & holdover core (a GPSDO building block).
//!
//! HAL-agnostic, integer-only, `no_std`, zero-dependency (by default) pure logic. It runs on
//! any MCU or host by just **passing/receiving integer-nanosecond timestamps** (runs on the
//! RP2040; equally on an STM32 timer input-capture or a host's `/dev/pps`).
//!
//! - [`DisciplinedClock`]: EMA-estimates the crystal frequency offset (ppb) from PPS intervals
//!   and keeps disciplined UTC, extrapolating through holdover while PPS is lost. Works with two
//!   timebases (capture/query).
//! - [`PpsTracker`]: classifies a PPS edge stream (lock / missed / non-monotonic).
//! - [`PpsTimeSync`]: pairs NMEA time with a PPS edge to establish a µs-precision UTC epoch.
//! - NMEA helpers ([`parse_rmc_time_date`] etc.): minimal time+date extraction (no full parse).
//!
//! The core does not require an NMEA parser itself ([`PpsTimeSync`] takes already-parsed values),
//! so you may use any NMEA parser (e.g. the [`nmea`](https://docs.rs/nmea) crate) alongside it.
//! The RP2040 firmware lives in the sibling `firmware/` crate (embassy-rp) in the same repo.
//!
//! `std` is enabled only under test; otherwise `no_std`.
//!
//! # Features
//!
//! - **`external-nmea`** (off by default): delegate [`parse_rmc_time_date`]'s RMC parsing to the
//!   [`nmea`](https://docs.rs/nmea) crate instead of the built-in parser. The default is the
//!   zero-dependency built-in parser.
//!
//!   Differences when enabled (measured on RP2040 / Cortex-M0+ @125MHz, see the firmware's
//!   `bench_nmea_parse`):
//!
//!   | | built-in (default) | `nmea` |
//!   |---|---|---|
//!   | time+date from one RMC | **~37 µs** | **~619 µs** (~17x slower) |
//!   | caller `.text` increase | 0 (~0.8 KB built-in) | **~+52 KB** (nom/chrono and ~6 crates) |
//!   | checksum validation | none | **yes** (mismatch → `None`) |
//!   | year interpretation | fixed 20xx | century pivot (`yy=94` → 1994) |
//!   | leap second `ss=60` | accepted (rolls into next minute) | rejected (`None`) |
//!
//!   The slowdown is ~4.3x on the host but widens on the M0+ (no FPU). It is negligible at 1 Hz
//!   either way (~0.06% CPU even with nmea). The default (built-in) is recommended; enable this
//!   only if you want checksum validation or already depend on `nmea`.
#![cfg_attr(not(test), no_std)]

mod assembler;
mod gpsdo;
mod pps;
mod timesync;

pub use assembler::{MAX_SENTENCE_LEN, NmeaLineAssembler};
pub use gpsdo::{DisciplinedClock, DisciplinedClockConfig, FreqUpdate, snap_to_second_ns};
pub use pps::{NOMINAL_US, PpsEvent, PpsTracker, PpsTrackerConfig, TOLERANCE_US};
pub use timesync::{
    PpsNmeaAssociation, PpsTimeSync, RmcTimeDate, SyncPoint, civil_to_unix, days_from_civil,
    parse_ddmmyy, parse_hhmmss, parse_rmc_time_date,
};
