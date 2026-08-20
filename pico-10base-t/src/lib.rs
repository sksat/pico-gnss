//! `pico-10base-t` — 10BASE-T Ethernet **transmit** from a Raspberry Pi Pico, with three resistors
//! and no PHY chip.
//!
//! A Rust port of [kingyoPiyo/Pico-10BASE-T](https://github.com/kingyoPiyo/Pico-10BASE-T) by kingyo
//! (MIT, Copyright (c) 2022 kingyo) — see the crate README. The PIO serialiser follows that design:
//! a `.side_set 2` program clocked at 20 MHz whose `out pc, 2` treats each 2-bit symbol as a jump
//! target, so the instruction slots *are* the line states and the CPU only hands over a pre-encoded
//! symbol stream.
//!
//! Layered the way `rp-pps` is: a HAL-agnostic core that runs under `cargo test` on the host, with
//! a thin backend behind a feature flag.
//!
//! - [`frame`] — Ethernet II / IPv4 / UDP framing and the Ethernet FCS. Pure integer logic, no HAL,
//!   no PIO. Host-testable, and cross-checked against Wireshark's dissectors.
//!
//! Transmit only, like upstream: receiving 10BASE-T needs hardware this wiring does not have.

#![no_std]

#[cfg(feature = "embassy-rp")]
pub mod embassy;
pub mod frame;
pub mod phy;

pub use frame::{Ipv4Addr, MacAddr, UdpFrameSpec, build_udp_frame};
pub use phy::{NLP_WORD, TP_IDL_WORD, encode_frame, encoded_words, manchester_word};
