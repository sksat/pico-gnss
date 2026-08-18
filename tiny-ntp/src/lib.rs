//! `tiny-ntp` — NTP (RFC 5905) for `no_std` targets.
//!
//! Integer-only, zero-dependency. This crate turns a **disciplined UTC instant (Unix
//! nanoseconds)** into the 48 bytes of an NTP packet, and back.
//!
//! It deliberately knows nothing about Ethernet, IPv4, UDP or any HAL — the only things crossing
//! into it are integer nanosecond timestamps and a UTC epoch, which is the same layer boundary the
//! rest of this workspace uses.
//!
//! **Scope today is one Stratum-1 server and the unicast client that can check it.** [`timestamp`]
//! and [`packet`] are what any NTP role needs; [`server`] answers only as a primary server, since
//! the stratum is not configurable and a secondary would have to accumulate an upstream path into
//! root delay. [`client::accept_broadcast`] is a stub — see its doc for the question that has to be
//! answered before it can be written.
//!
//! - [`timestamp`] — NTP's two fixed-point time formats and their Unix-ns conversions.
//! - [`packet`] — the 48-byte header and its wire encoding.
//! - [`client`] — building a request and turning a reply into an offset and a delay.
//! - [`server`] — Stratum-1 policy: when we may serve at all, and what to claim. Both service
//!   modes are here — [`server::respond`] for a unicast client exchange and [`server::broadcast`]
//!   for one-way announcement — since they differ only in which timestamps are meaningful.
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

pub mod client;
pub mod packet;
pub mod server;
pub mod timestamp;

pub use packet::{LeapIndicator, Mode, NtpPacket, PACKET_LEN};
pub use server::{ClockState, ServeDecision, ServerConfig, SilentReason, broadcast, respond};
pub use timestamp::{NTP_UNIX_OFFSET_SECS, NtpShort, NtpTimestamp};
