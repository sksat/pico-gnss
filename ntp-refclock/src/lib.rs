//! `ntp-refclock` — portable NTP (RFC 5905) **reference-clock server** core.
//!
//! `no_std`, integer-only, zero-dependency. This crate turns a **disciplined UTC instant (Unix
//! nanoseconds)** into the 48 bytes of an NTP packet, and back. It deliberately knows nothing about
//! Ethernet, IPv4, UDP or any HAL — the only things crossing into it are integer nanosecond
//! timestamps and a UTC epoch, which is the same layer boundary the rest of this workspace uses.
//!
//! - [`timestamp`] — NTP's two fixed-point time formats and their Unix-ns conversions.
//!
//! # Where the other pieces live
//!
//! | Concern | Crate |
//! |---|---|
//! | Disciplined UTC + holdover (the time source) | [`gnssdo`](https://docs.rs/gnssdo) |
//! | Ethernet / IPv4 / UDP framing, 10BASE-T PHY | `pico-10base-t` |
//! | Wiring it together on real hardware | `pico-ntp` |
//!
//! Keeping framing out of this crate is what lets the NTP layer stay L2/L3/L4-agnostic: the same
//! packet bytes go out over 10BASE-T here, or over anything else elsewhere.

#![no_std]

pub mod timestamp;

pub use timestamp::{NTP_UNIX_OFFSET_SECS, NtpShort, NtpTimestamp};
