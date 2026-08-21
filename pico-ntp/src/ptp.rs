//! What both boards need to speak PTP over this link.
//!
//! The protocol itself is `tiny-ptp`, which knows only integers. What is here is the part that is
//! specific to these two boards: which UDP port the messages travel on, who each end says it is,
//! and — the only interesting one — how a counter value taken at a pin becomes a UTC nanosecond.
//!
//! **That conversion is the whole reason this exists.** A `Sync` leaving the master is timestamped
//! by a state machine watching GP16, which reports a 32-bit down-counter value and nothing else.
//! Turning that into the UTC the message has to carry takes two steps, and both are already built:
//! [`rp_pps::TickTimeline`] anchored at the counter's start puts the value on the same scale as the
//! 1PPS capture, and the clock that scale feeds maps it to UTC. No software timestamp anywhere on
//! the path — which is the difference between this and the NTP the same firmware also serves.

use pico_10base_t::frame::{Ipv4Addr, MacAddr, UdpFrameSpec, build_udp_frame};
use rp_pps::{TickTimeline, ticks_to_ns};
use tiny_ptp::{ClockIdentity, MAX_MESSAGE_LEN, Message, PortIdentity, encode};

/// The port the exchange travels on.
///
/// IEEE 1588 splits its messages across 319 (event, the ones whose timestamps matter) and 320
/// (general). Both ends here are one task apiece and neither prioritises by port, so splitting
/// would add a second socket and change nothing. `PTP_DST_PORT` overrides it for the same reason
/// `NTP_DST_PORT` exists: to run beside something already using the real one.
pub const PTP_PORT: u16 = match option_env!("PTP_DST_PORT") {
    Some(s) => crate::parse_port(s),
    None => 319,
};

/// How often the master sends a `Sync`, as the standard writes intervals: log₂ seconds.
pub const LOG_SYNC_INTERVAL: i8 = 0;

/// The PTP domain. Zero is the default domain, and there is only one here.
pub const DOMAIN: u8 = 0;

/// A port identity built from a MAC, the way IEEE 1588 says to: the six bytes with `FF FE` in the
/// middle, and port 1 because each board has one.
pub fn port_identity(mac: MacAddr) -> PortIdentity {
    PortIdentity {
        clock: ClockIdentity::from_mac(mac.0),
        port: 1,
    }
}

/// A counter value from an [`rp_pps::embassy::EventCapture`], as nanoseconds since the counters
/// started.
///
/// The timeline has to be the one anchored at the start
/// ([`TickTimeline::from_counter_start_with_toll`]); one anchored at its first capture counts from
/// a different origin, and the number would be an interval wearing a timestamp's clothes.
pub fn capture_ns(raw: u32, timeline: &mut TickTimeline, clk_hz: u32) -> u64 {
    ticks_to_ns(timeline.observe(raw), clk_hz)
}

/// Put one PTP message into an Ethernet frame.
pub struct Peer {
    pub src_mac: MacAddr,
    pub dst_mac: MacAddr,
    pub src_ip: Ipv4Addr,
    pub dst_ip: Ipv4Addr,
}

/// Build the frame for `msg`, returning how long it is.
pub fn build(peer: &Peer, msg: &Message, ip_id: u16, frame: &mut [u8]) -> Option<usize> {
    let mut payload = [0u8; MAX_MESSAGE_LEN];
    let len = encode(msg, &mut payload)?;
    build_udp_frame(
        &UdpFrameSpec {
            src_mac: peer.src_mac,
            dst_mac: peer.dst_mac,
            src_ip: peer.src_ip,
            dst_ip: peer.dst_ip,
            src_port: PTP_PORT,
            dst_port: PTP_PORT,
            ip_id,
            ttl: 1,
            payload: &payload[..len],
        },
        frame,
    )
}
