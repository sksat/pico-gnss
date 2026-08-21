//! An end-to-end two-step PTP exchange, and nothing else.
//!
//! Two boards, one wire, and fixed roles. That is the whole of what this crate is for, so what it
//! implements is the part of IEEE 1588 that carries time between them — `Sync`, `Follow_Up`,
//! `Delay_Req`, `Delay_Resp`, and the offset and path delay they produce — and it stops there. No
//! best-master algorithm, no `Announce`, no management, no peer delay: with two ports that were
//! built together there is nothing to elect and nothing to discover. What it should be called is a
//! static-role subset, not an ordinary clock.
//!
//! **Why two-step, when one-step is simpler.** A transmit timestamp has to be written into a
//! message before the message is checksummed and encoded, so it is always a claim about a moment
//! that has not happened. On this hardware that claim was wrong by hundreds of microseconds and by
//! an amount that varied with how long the encoding took — measured, on the NTP path this replaces.
//! Two-step is the standard's answer: send the message, let the hardware say when it actually left,
//! and send that afterwards in a `Follow_Up`. The timestamp that counts is never inside the message
//! it describes.
//!
//! The boundary is `tiny-ntp`'s. Nothing here knows about Ethernet, UDP, or where a timestamp came
//! from; the caller brings integers and gets integers back. `no_std`, no dependencies.

#![no_std]
#![forbid(unsafe_code)]

pub mod e2e;
pub mod message;

pub use e2e::{Exchange, Measurement, Reject, measure};
pub use message::{
    Body, ClockIdentity, HEADER_LEN, MAX_MESSAGE_LEN, Message, MessageType, PortIdentity,
    Timestamp, VERSION, decode, encode,
};
