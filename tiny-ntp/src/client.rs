//! The client half of the on-wire protocol: ask a server, and turn its reply into an offset.
//!
//! Nothing here touches a socket. [`request`] hands back the packet to send and [`measure`] takes
//! the reply plus the two timestamps only the client can know — when it sent, and when the answer
//! came back — which is the same layer boundary [`crate::server`] sits on.

use crate::packet::{LeapIndicator, Mode, NtpPacket};
use crate::server::NTP_VERSION;
use crate::timestamp::{NtpShort, NtpTimestamp};

/// Why a reply was thrown away.
///
/// RFC 5905 §8 calls these the packet sanity checks. They are not paranoia: a client that skips
/// them will happily set its clock from a stale duplicate, or from a server that is announcing it
/// has no idea what time it is.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Reject {
    /// Not a server's answer (mode 4). A broadcast has no origin timestamp to match against, so it
    /// cannot be measured this way at all — use [`accept_broadcast`].
    NotAReply,
    /// Not a broadcast (mode 5), passed to [`accept_broadcast`].
    NotABroadcast,
    /// Answered in a different version than we asked in.
    VersionMismatch,
    /// The echoed origin timestamp is not the one we sent, so this reply belongs to some other
    /// request — or to nobody. RFC 5905 calls such a packet *bogus*.
    Bogus,
    /// The server says it is not synchronised, by leap indicator or by stratum. Stratum 0 is a
    /// kiss-o'-death; 16 and above is unsynchronised.
    Unsynchronized,
}

/// What one exchange measured.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Measurement {
    /// How far the server's clock is ahead of ours (ns). Add it to local time to get server time.
    pub offset_ns: i64,
    /// Round-trip delay with the server's own processing time removed (ns).
    ///
    /// Can come out negative on a clock whose resolution is coarser than the network is fast; RFC
    /// 5905 keeps the sign rather than clamping, since a negative delay is evidence about the
    /// timestamps rather than about the path.
    pub delay_ns: i64,
    /// The stratum the server claimed. 1 is a primary server, i.e. one with a reference clock.
    pub stratum: u8,
    /// The server's own uncertainty (ns), from the root dispersion field.
    pub root_dispersion_ns: u64,
}

/// Build a mode-3 request that departs at `transmit_unix_ns`.
///
/// The transmit timestamp is the whole of the client's state for this exchange: the server echoes
/// it back untouched, and [`measure`] matches on it. Everything else a client sends is ignored by
/// a server that is only being asked the time.
pub fn request(transmit_unix_ns: i64, poll: i8) -> NtpPacket {
    NtpPacket {
        leap: LeapIndicator::NoWarning,
        version: NTP_VERSION,
        mode: Mode::Client,
        // A client has nothing to say about its own clock, and RFC 5905 §7.3 has it send zeros.
        stratum: 0,
        poll,
        precision: 0,
        root_delay: NtpShort::ZERO,
        root_dispersion: NtpShort::ZERO,
        reference_id: [0; 4],
        reference_timestamp: NtpTimestamp::ZERO,
        origin_timestamp: NtpTimestamp::ZERO,
        receive_timestamp: NtpTimestamp::ZERO,
        transmit_timestamp: NtpTimestamp::from_unix_ns(transmit_unix_ns),
    }
}

/// Offset and delay from one request/reply pair (RFC 5905 §8).
///
/// `destination_unix_ns` is when the reply arrived by the local clock. The other three timestamps
/// come off the wire, and are resolved into Unix nanoseconds against that arrival: all four sit
/// within one round trip of each other, so it is the era the reply belongs to.
pub fn measure(
    request: &NtpPacket,
    reply: &NtpPacket,
    destination_unix_ns: i64,
) -> Result<Measurement, Reject> {
    if reply.mode != Mode::Server {
        return Err(Reject::NotAReply);
    }
    if reply.version != request.version {
        return Err(Reject::VersionMismatch);
    }
    // Both directions matter. An unset origin means the server never saw our request; a different
    // one means we are looking at the answer to someone else's.
    if reply.origin_timestamp != request.transmit_timestamp
        || reply.transmit_timestamp == NtpTimestamp::ZERO
    {
        return Err(Reject::Bogus);
    }
    if reply.leap == LeapIndicator::Unsynchronized || reply.stratum == 0 || reply.stratum >= 16 {
        return Err(Reject::Unsynchronized);
    }

    let t1 = request
        .transmit_timestamp
        .to_unix_ns_near(destination_unix_ns);
    let t2 = reply.receive_timestamp.to_unix_ns_near(destination_unix_ns);
    let t3 = reply
        .transmit_timestamp
        .to_unix_ns_near(destination_unix_ns);
    let t4 = destination_unix_ns;

    // In i128 throughout. The differences are small in any sane exchange, but the inputs are
    // attacker-supplied: a reply claiming 1900 against a local clock in 2036 would overflow i64
    // halfway through, and a wrapped offset is one that gets *applied*.
    let offset = ((t2 as i128 - t1 as i128) + (t3 as i128 - t4 as i128)) / 2;
    let delay = (t4 as i128 - t1 as i128) - (t3 as i128 - t2 as i128);

    Ok(Measurement {
        offset_ns: offset.clamp(i64::MIN as i128, i64::MAX as i128) as i64,
        delay_ns: delay.clamp(i64::MIN as i128, i64::MAX as i128) as i64,
        stratum: reply.stratum,
        root_dispersion_ns: reply.root_dispersion.to_nanos(),
    })
}

/// Take the time from a one-way [`Mode::Broadcast`] packet (RFC 5905 §9.1).
///
/// A broadcast carries only its own departure, so the arithmetic that separates offset from path
/// delay in [`measure`] has nothing to work with: `T3 - T4` is the offset *minus* the one-way
/// delay, and no amount of listening will tell them apart. `delay_ns` is the round-trip delay the
/// caller believes in, half of which is added back.
///
/// Where that number comes from is the caller's problem, and RFC 5905 §9.1 answers it by running a
/// short volley of ordinary [`request`]/[`measure`] exchanges first and reusing the delay they
/// measured. A server that cannot receive — as this workspace's transmit-only PHY cannot — leaves
/// its clients to configure the delay instead, and to wear the error if it is wrong. Passing `0` is
/// legitimate: it says the path is short compared to the accuracy wanted.
///
/// Stateless, so it cannot see replays. A broadcast is trivially forged and trivially repeated, and
/// RFC 5905 §9.1 has the client remember the last transmit timestamp it accepted and discard
/// anything not newer. That state belongs to the caller, along with the decision of whom to believe.
pub fn accept_broadcast(
    broadcast: &NtpPacket,
    destination_unix_ns: i64,
    delay_ns: i64,
) -> Result<Measurement, Reject> {
    if broadcast.mode != Mode::Broadcast {
        return Err(Reject::NotABroadcast);
    }
    if broadcast.transmit_timestamp == NtpTimestamp::ZERO {
        return Err(Reject::Bogus);
    }
    if broadcast.leap == LeapIndicator::Unsynchronized
        || broadcast.stratum == 0
        || broadcast.stratum >= 16
    {
        return Err(Reject::Unsynchronized);
    }

    let t3 = broadcast
        .transmit_timestamp
        .to_unix_ns_near(destination_unix_ns);
    // i128 for the same reason as `measure`: the departure time is attacker-supplied.
    let offset = (t3 as i128 - destination_unix_ns as i128) + (delay_ns as i128) / 2;

    Ok(Measurement {
        offset_ns: offset.clamp(i64::MIN as i128, i64::MAX as i128) as i64,
        delay_ns,
        stratum: broadcast.stratum,
        root_dispersion_ns: broadcast.root_dispersion.to_nanos(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::server::{self, ClockState, LeapWarning, ServeDecision, ServerConfig, Source};

    const T: i64 = 1_787_020_967 * 1_000_000_000;

    fn cfg() -> ServerConfig {
        ServerConfig {
            precision: -20,
            poll: 4,
            source: Source::ReferenceClock { id: *b"GPS\0" },
            base_dispersion_ns: 1_000_000,
            holdover_drift_ppb: 100,
            max_holdover_ns: 3_600 * 1_000_000_000,
        }
    }

    fn locked(now: i64) -> ClockState {
        ClockState {
            last_update_unix_ns: Some(now),
            holdover_ns: 0,
            frequency_locked: true,
            leap: LeapWarning::None,
        }
    }

    #[test]
    fn a_secondary_server_is_a_usable_source() {
        // The sanity checks reject stratum 0 and 16, so they must not reject what lies between:
        // most of the servers a client will ever talk to are secondaries.
        let secondary = ServerConfig {
            source: Source::Upstream {
                address: [10, 0, 0, 1],
                stratum: 1,
                root_delay: NtpShort::from_nanos(20_000_000),
                root_dispersion: NtpShort::from_nanos(30_000_000),
                delay_ns: 6_000_000,
            },
            ..cfg()
        };
        let req = request(T, 4);
        let ServeDecision::Serve(reply) = server::respond(&secondary, &locked(T), &req, T, T)
        else {
            panic!("a locked clock must answer");
        };
        let m = measure(&req, &reply, T).expect("stratum 2 is synchronised");
        assert_eq!(m.stratum, 2);
        // The dispersion a client sees is the whole path's, not just the server's own.
        assert!(
            m.root_dispersion_ns >= 30_000_000,
            "the upstream's dispersion has to survive the trip: {}",
            m.root_dispersion_ns
        );
    }

    /// Run one exchange where the server's clock leads ours by `offset_ns` and each direction of
    /// the path costs `one_way_ns`.
    fn exchange(offset_ns: i64, one_way_ns: i64, server_processing_ns: i64) -> Measurement {
        let t1 = T;
        let req = request(t1, 4);
        // Server-side timestamps are on the server's clock, which is `offset_ns` ahead.
        let t2 = t1 + one_way_ns + offset_ns;
        let t3 = t2 + server_processing_ns;
        let t4 = t3 - offset_ns + one_way_ns;

        let ServeDecision::Serve(reply) = server::respond(&cfg(), &locked(t2), &req, t2, t3) else {
            panic!("a locked clock must answer");
        };
        measure(&req, &reply, t4).expect("our own server's reply must pass the sanity checks")
    }

    #[test]
    fn recovers_the_offset_a_symmetric_path_hides() {
        let m = exchange(250_000_000, 5_000_000, 1_000_000);
        assert_eq!(m.offset_ns, 250_000_000);
        assert_eq!(m.delay_ns, 10_000_000);
        assert_eq!(m.stratum, 1);
    }

    #[test]
    fn server_processing_time_is_not_charged_to_the_path() {
        // A slow server must not look like a distant one.
        let quick = exchange(0, 5_000_000, 1_000_000);
        let slow = exchange(0, 5_000_000, 900_000_000);
        assert_eq!(quick.delay_ns, slow.delay_ns);
        assert_eq!(slow.offset_ns, 0);
    }

    #[test]
    fn an_asymmetric_path_biases_the_offset_by_half_the_difference() {
        // The one error NTP cannot see. Recorded here so that a change in the arithmetic which
        // *removed* this bias would be caught as the mistake it is.
        let t1 = T;
        let req = request(t1, 4);
        let (out_ns, back_ns) = (2_000_000, 8_000_000);
        let t2 = t1 + out_ns;
        let t3 = t2;
        let t4 = t3 + back_ns;
        let ServeDecision::Serve(reply) = server::respond(&cfg(), &locked(t2), &req, t2, t3) else {
            panic!("a locked clock must answer");
        };
        let m = measure(&req, &reply, t4).unwrap();
        assert_eq!(m.offset_ns, (out_ns - back_ns) / 2);
        assert_eq!(m.delay_ns, out_ns + back_ns);
    }

    #[test]
    fn a_reply_to_someone_elses_request_is_bogus() {
        let req = request(T, 4);
        let other = request(T + 1, 4);
        let ServeDecision::Serve(reply) = server::respond(&cfg(), &locked(T), &other, T, T) else {
            panic!("a locked clock must answer");
        };
        assert_eq!(measure(&req, &reply, T), Err(Reject::Bogus));
    }

    #[test]
    fn a_broadcast_is_not_an_answer() {
        let req = request(T, 4);
        let ServeDecision::Serve(bcast) = server::broadcast(&cfg(), &locked(T), T) else {
            panic!("a locked clock must answer");
        };
        assert_eq!(measure(&req, &bcast, T), Err(Reject::NotAReply));
    }

    #[test]
    fn an_unsynchronised_server_is_refused_even_though_it_answered() {
        let req = request(T, 4);
        let ServeDecision::Serve(mut reply) = server::respond(&cfg(), &locked(T), &req, T, T)
        else {
            panic!("a locked clock must answer");
        };
        reply.leap = LeapIndicator::Unsynchronized;
        assert_eq!(measure(&req, &reply, T), Err(Reject::Unsynchronized));

        let ServeDecision::Serve(mut reply) = server::respond(&cfg(), &locked(T), &req, T, T)
        else {
            panic!("a locked clock must answer");
        };
        reply.stratum = 16;
        assert_eq!(measure(&req, &reply, T), Err(Reject::Unsynchronized));
    }

    #[test]
    fn a_version_we_did_not_ask_in_is_refused() {
        let mut req = request(T, 4);
        req.version = 3;
        let ServeDecision::Serve(mut reply) = server::respond(&cfg(), &locked(T), &req, T, T)
        else {
            panic!("a locked clock must answer");
        };
        reply.version = 4;
        assert_eq!(measure(&req, &reply, T), Err(Reject::VersionMismatch));
    }

    #[test]
    fn a_broadcast_gives_the_offset_once_the_delay_is_supplied() {
        let (offset_ns, one_way_ns) = (250_000_000, 5_000_000);
        // The server stamps departure on its own clock, which leads ours.
        let t3 = T + offset_ns;
        let t4 = T + one_way_ns;
        let ServeDecision::Serve(bcast) = server::broadcast(&cfg(), &locked(t3), t3) else {
            panic!("a locked clock must announce");
        };
        let m = accept_broadcast(&bcast, t4, 2 * one_way_ns).unwrap();
        assert_eq!(m.offset_ns, offset_ns);
        assert_eq!(m.delay_ns, 2 * one_way_ns);
        assert_eq!(m.stratum, 1);
    }

    #[test]
    fn a_broadcast_client_that_assumes_no_delay_is_wrong_by_the_one_way_time() {
        // What a caller with nothing to calibrate against is choosing. Worth being explicit that
        // the error is the one-way time and not something smaller.
        let one_way_ns = 5_000_000;
        let ServeDecision::Serve(bcast) = server::broadcast(&cfg(), &locked(T), T) else {
            panic!("a locked clock must announce");
        };
        let m = accept_broadcast(&bcast, T + one_way_ns, 0).unwrap();
        assert_eq!(m.offset_ns, -one_way_ns);
    }

    #[test]
    fn a_unicast_reply_is_not_a_broadcast() {
        let req = request(T, 4);
        let ServeDecision::Serve(reply) = server::respond(&cfg(), &locked(T), &req, T, T) else {
            panic!("a locked clock must answer");
        };
        assert_eq!(
            accept_broadcast(&reply, T, 0),
            Err(Reject::NotABroadcast),
            "mode 4 carries an origin to match; routing it here would skip that check"
        );
    }

    #[test]
    fn an_unsynchronised_broadcast_is_refused() {
        let ServeDecision::Serve(mut bcast) = server::broadcast(&cfg(), &locked(T), T) else {
            panic!("a locked clock must announce");
        };
        bcast.leap = LeapIndicator::Unsynchronized;
        assert_eq!(accept_broadcast(&bcast, T, 0), Err(Reject::Unsynchronized));
    }

    #[test]
    fn survives_a_reply_from_the_far_side_of_the_era_boundary() {
        // 32-bit NTP seconds wrap in 2036. A hostile or broken reply on the wrong side of it must
        // not produce a wrapped offset, because an offset is a number that gets applied.
        let req = request(T, 4);
        let mut reply = request(T, 4);
        reply.mode = Mode::Server;
        reply.stratum = 1;
        reply.origin_timestamp = req.transmit_timestamp;
        reply.receive_timestamp = NtpTimestamp::from_bits(u64::MAX);
        reply.transmit_timestamp = NtpTimestamp::from_bits(u64::MAX);
        let m = measure(&req, &reply, T).expect("sanity checks pass; only the arithmetic is odd");
        assert!(m.offset_ns.checked_abs().is_some(), "no i64 overflow");
    }
}
