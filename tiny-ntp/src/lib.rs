//! `tiny-ntp` — NTP (RFC 5905) for `no_std` targets.
//!
//! Integer-only, zero-dependency. This crate turns a **disciplined UTC instant (Unix
//! nanoseconds)** into the 48 bytes of an NTP packet, and back.
//!
//! It deliberately knows nothing about Ethernet, IPv4, UDP or any HAL — the only things crossing
//! into it are integer nanosecond timestamps and a UTC epoch, which is the same layer boundary the
//! rest of this workspace uses.
//!
//! [`server`] answers as either a primary server holding a reference clock or a secondary following
//! another server, and [`client`] covers both service modes — a unicast exchange, and the one-way
//! broadcast that a transmit-only server leaves as the only option.
//!
//! What it does not do is choose. RFC 5905 §10-11 — polling several servers, filtering their
//! samples and picking whom to believe — is an algorithm with state and a scheduler, and it belongs
//! above this layer rather than inside it.
//!
//! - [`timestamp`] — NTP's two fixed-point time formats and their Unix-ns conversions.
//! - [`packet`] — the 48-byte header and its wire encoding.
//! - [`client`] — building a request, and turning a reply or a broadcast into an offset.
//! - [`server`] — server policy: when we may serve at all, and what to claim. Both service
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
pub mod discipline;
pub mod packet;
pub mod server;
pub mod timestamp;

pub use packet::{LeapIndicator, Mode, NtpPacket, PACKET_LEN};
pub use server::{ClockState, ServeDecision, ServerConfig, SilentReason, broadcast, respond};
pub use timestamp::{NTP_UNIX_OFFSET_SECS, NtpShort, NtpTimestamp};
