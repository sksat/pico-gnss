//! `gnssdo`: a GNSS-PPS-disciplined clock & holdover core (a GPSDO building block).
//!
//! HAL-agnostic, integer-only, `no_std`, zero-dependency pure logic. It disciplines a local
//! oscillator against a GNSS 1PPS reference: estimate the crystal's frequency offset, hold over
//! through PPS loss, keep disciplined UTC, and phase-lock a generated output. It runs on any MCU
//! or host by just **passing integer-nanosecond timestamps and a UTC epoch** (on the RP2040's PIO
//! capture, an STM32 timer input-capture, or a host's `/dev/pps` — all the same).
//!
//! - [`DisciplinedClock`]: EMA-estimates the crystal frequency offset (ppb) from PPS intervals and
//!   keeps disciplined UTC, extrapolating through holdover while PPS is lost. Works with two
//!   timebases (capture/query); fed an epoch via [`DisciplinedClock::update_epoch`].
//! - [`PpsTracker`]: classifies a PPS edge stream (lock / missed / non-monotonic).
//! - [`PhaseLockLoop`]: a type-II output-phase servo (P/I/D + Smith predictor) for disciplining a
//!   *generated* 1PPS edge to the reference.
//! - [`Gnssdo`]: an all-in-one easy tier bundling [`PpsTracker`] + [`DisciplinedClock`] under one good
//!   default discipline policy.
//!
//! # Scope
//!
//! The core is agnostic to **where** absolute time comes from: it consumes an epoch
//! (`capture_ns ↔ unix_ns`) via [`DisciplinedClock::update_epoch`] / [`Gnssdo::on_utc`]. Decoding
//! the time source (NMEA framing/parsing) and pairing a PPS edge with its UTC second are a separate
//! responsibility and live in the sibling `rp-pps` crate, not here.
#![cfg_attr(not(test), no_std)]

mod gpsdo;
mod pll;
mod pps;
mod qerr;

pub use gpsdo::{
    DisciplinedClock, DisciplinedClockConfig, FreqUpdate, Gnssdo, GnssdoStep, snap_to_second_ns,
};
pub use pll::{LoopMode, PhaseLockLoop, PhaseLockLoopConfig, PhaseLockLoopUpdate};
pub use pps::{NOMINAL_US, PpsEvent, PpsTracker, PpsTrackerConfig, TOLERANCE_US};
pub use qerr::{QErrCorrector, correct_phase_ns};
