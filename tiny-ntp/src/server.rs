//! Server policy: deciding *whether* we may serve the time, and what to claim about it.
//!
//! The wire format ([`crate::packet`]) is neutral about where a server's time came from. This
//! module is where that shows up: refusing to transmit while undisciplined, growing root dispersion
//! through a holdover, and filling in the stratum, the reference identifier and the accumulated
//! path back to the root — all of which differ between a server holding a [reference
//! clock](Source::ReferenceClock) and one [following another server](Source::Upstream).
//!
//! It takes the clock's state as plain integers, so it is testable on the host without a GNSS
//! receiver anywhere nearby.

use crate::packet::{LeapIndicator, Mode, NtpPacket};
use crate::timestamp::{NtpShort, NtpTimestamp};

/// The deepest stratum that still means "synchronised" (RFC 5905 §7.3: MAXSTRAT).
pub const MAX_STRATUM: u8 = 16;

/// Where this server's time comes from, which is what decides its stratum and what it advertises
/// as the path back to the root.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Source {
    /// Hardware attached to this machine: a primary server, stratum 1.
    ReferenceClock {
        /// Four-character ASCII source code (RFC 5905 §7.3), e.g. `GPS\0`.
        id: [u8; 4],
    },
    /// Another NTP server, making this a secondary server one stratum below it.
    Upstream {
        /// The upstream's IPv4 address, which is what the reference identifier carries for IPv4
        /// (RFC 5905 §7.3). A secondary names its source by address, not by a text code.
        address: [u8; 4],
        /// The upstream's own stratum. Ours is one more.
        stratum: u8,
        /// The upstream's root delay, which is its distance to the root and not to us.
        root_delay: NtpShort,
        /// The upstream's root dispersion, likewise.
        root_dispersion: NtpShort,
        /// Round-trip delay to the upstream, as [`crate::client::measure`] reports it. Added to the
        /// upstream's root delay, since our clients are that much further from the root than we are.
        delay_ns: i64,
    },
}

/// A leap second the time source has announced for the end of the current UTC day.
///
/// Deliberately not [`LeapIndicator`], which also has a value meaning *unsynchronised*. Whether we
/// are synchronised is decided by [`ClockState`] and the gate below, not by whoever fills this in;
/// letting a caller set it here would be a second, contradictory way to answer the same question.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum LeapWarning {
    #[default]
    None,
    /// The last minute of the day will have 61 seconds.
    Insert,
    /// The last minute of the day will have 59 seconds.
    Delete,
}

/// Static description of this server.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ServerConfig {
    /// log2 seconds of the resolution at which we can actually *timestamp* a transmission.
    ///
    /// This is not the oscillator's precision. Claiming better than we can timestamp corrupts a
    /// client's source selection, so it must come from measurement of the transmit path.
    pub precision: i8,
    /// log2 seconds between broadcasts.
    pub poll: i8,
    /// What we are synchronised to, which sets our stratum and reference identifier.
    pub source: Source,
    /// Our own uncertainty when freshly disciplined (ns) — the floor of root dispersion.
    pub base_dispersion_ns: u64,
    /// Bound on fractional frequency error (ppb) used to grow dispersion during holdover.
    pub holdover_drift_ppb: u64,
    /// Stop serving once holdover exceeds this (ns).
    pub max_holdover_ns: u64,
}

/// What the discipline core says about the clock right now. Mirrors what `gnssdo` can answer.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ClockState {
    /// UTC of the last discipline update — NTP's "reference timestamp". `None` before the first
    /// PPS↔UTC pairing, i.e. we do not know what time it is at all.
    pub last_update_unix_ns: Option<i64>,
    /// Time since that update (ns). Non-zero means we are extrapolating.
    pub holdover_ns: u64,
    /// Whether the frequency estimate has locked. Without it, holdover extrapolation is meaningless.
    pub frequency_locked: bool,
    /// A leap second the source has announced, passed on to clients so they can apply it too.
    pub leap: LeapWarning,
}

/// Why we are staying silent.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SilentReason {
    /// No UTC epoch yet — we do not know the time.
    NoEpoch,
    /// The frequency estimate has not locked, so we cannot bound our own drift.
    FrequencyUnlocked,
    /// Holdover has run longer than the configured limit.
    HoldoverExceeded,
    /// Following this upstream would put us at [`MAX_STRATUM`] or deeper, which on the wire means
    /// unsynchronised. Serving it would be claiming to be a source while announcing we are not one.
    TooDeep,
    /// What arrived was not a client asking the time (RFC 5905 mode 3). Only [`respond`] reports
    /// this; nothing about our own clock is wrong.
    NotARequest,
    /// A client asked in a version this crate does not speak.
    UnsupportedVersion,
}

/// Whether to transmit, and what.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ServeDecision {
    /// Transmit this packet.
    Serve(NtpPacket),
    /// Transmit nothing.
    ///
    /// Deliberately silent rather than sending `LI=3, stratum=16`: a one-way broadcast client has
    /// no way to interrogate us, and an unsynchronised beacon is only noise on the wire.
    Silent(SilentReason),
}

/// The NTP version this crate speaks.
pub const NTP_VERSION: u8 = 4;

/// Root dispersion (ns) for a given holdover: our floor, plus what the frequency bound could have
/// accumulated since the last discipline update.
///
/// Computed in `u128` and saturating at every step. Wrapping here would be worse than useless: a
/// wrapped dispersion advertises a *better* clock than we have, and clients would believe it.
pub fn root_dispersion_ns(cfg: &ServerConfig, holdover_ns: u64) -> u64 {
    let growth =
        (holdover_ns as u128).saturating_mul(cfg.holdover_drift_ppb as u128) / 1_000_000_000;
    let growth = if growth > u64::MAX as u128 {
        u64::MAX
    } else {
        growth as u64
    };
    cfg.base_dispersion_ns.saturating_add(growth)
}

/// Build the broadcast packet for a transmission scheduled at `transmit_unix_ns`.
///
/// The transmit timestamp is **passed in, not read from a clock**: for a one-way broadcast the
/// client cannot measure the path, so the value must describe when the frame actually leaves,
/// which is decided by the transmit schedule rather than by when this function ran.
/// The gates every service mode shares. `Ok` carries the UTC of the last discipline update, which
/// becomes the reference timestamp.
///
/// Order matters: not knowing the time at all is a different (and prior) failure to knowing it but
/// being unable to bound how fast we are losing it.
fn gate(cfg: &ServerConfig, state: &ClockState) -> Result<i64, SilentReason> {
    let Some(last_update) = state.last_update_unix_ns else {
        return Err(SilentReason::NoEpoch);
    };
    if !state.frequency_locked {
        return Err(SilentReason::FrequencyUnlocked);
    }
    if state.holdover_ns > cfg.max_holdover_ns {
        return Err(SilentReason::HoldoverExceeded);
    }
    if stratum(&cfg.source) >= MAX_STRATUM {
        return Err(SilentReason::TooDeep);
    }
    Ok(last_update)
}

/// Whether the clock is in a state we may serve from, without building a packet to find out.
///
/// For callers that prepare a frame ahead of the instant it is due to leave: encoding takes time,
/// and doing it after the transmit timestamp has passed makes that timestamp a lie. Build early,
/// then ask this immediately before handing the buffer over — the answer can change in between, and
/// what matters is the answer at transmission.
pub fn may_serve(cfg: &ServerConfig, state: &ClockState) -> Result<(), SilentReason> {
    gate(cfg, state).map(|_| ())
}

/// Our stratum: 1 for a reference clock, one more than the upstream otherwise.
///
/// Saturating, so a chain already at the bottom stays there rather than wrapping to 0 — which is a
/// kiss-o'-death and would be read as something else entirely.
fn stratum(source: &Source) -> u8 {
    match source {
        Source::ReferenceClock { .. } => 1,
        Source::Upstream { stratum, .. } => stratum.saturating_add(1),
    }
}

/// The fields that describe *this server* rather than the exchange, shared by both service modes.
fn base_packet(cfg: &ServerConfig, state: &ClockState, last_update: i64) -> NtpPacket {
    let own_dispersion = root_dispersion_ns(cfg, state.holdover_ns);
    // What a client is told about the whole path to the root, not just to us. For a reference
    // clock both are ours alone; for a secondary, the upstream's figures are already in the packet
    // it sent us and our own hop goes on top.
    let (reference_id, root_delay, root_dispersion) = match cfg.source {
        // Stratum 1 *is* the root: there is no upstream path to have delay to.
        Source::ReferenceClock { id } => (id, NtpShort::ZERO, NtpShort::from_nanos(own_dispersion)),
        Source::Upstream {
            address,
            root_delay,
            root_dispersion,
            delay_ns,
            ..
        } => (
            address,
            NtpShort::from_nanos(
                root_delay
                    .to_nanos()
                    .saturating_add(delay_ns.unsigned_abs()),
            ),
            NtpShort::from_nanos(root_dispersion.to_nanos().saturating_add(own_dispersion)),
        ),
    };
    NtpPacket {
        leap: match state.leap {
            LeapWarning::None => LeapIndicator::NoWarning,
            LeapWarning::Insert => LeapIndicator::LastMinute61,
            LeapWarning::Delete => LeapIndicator::LastMinute59,
        },
        version: NTP_VERSION,
        mode: Mode::Broadcast,
        stratum: stratum(&cfg.source),
        poll: cfg.poll,
        precision: cfg.precision,
        root_delay,
        root_dispersion,
        reference_id,
        reference_timestamp: NtpTimestamp::from_unix_ns(last_update),
        origin_timestamp: NtpTimestamp::ZERO,
        receive_timestamp: NtpTimestamp::ZERO,
        transmit_timestamp: NtpTimestamp::ZERO,
    }
}

pub fn broadcast(cfg: &ServerConfig, state: &ClockState, transmit_unix_ns: i64) -> ServeDecision {
    let last_update = match gate(cfg, state) {
        Ok(t) => t,
        Err(reason) => return ServeDecision::Silent(reason),
    };
    ServeDecision::Serve(NtpPacket {
        mode: Mode::Broadcast,
        // A broadcast answers no request, so there is no origin to echo and no arrival to report.
        transmit_timestamp: NtpTimestamp::from_unix_ns(transmit_unix_ns),
        ..base_packet(cfg, state, last_update)
    })
}

/// Build a reply to a client's request (RFC 5905 mode 3 → mode 4).
///
/// `receive_unix_ns` is when the request arrived and `transmit_unix_ns` when the reply will leave;
/// together with the client's own two timestamps they are what lets the client separate offset from
/// round-trip delay — the thing a one-way [`broadcast`] can never give it.
///
/// Gated identically to [`broadcast`]: an undisciplined clock answers nothing. Additionally, only a
/// mode-3 request in a version we speak gets an answer — see [`SilentReason::NotARequest`].
pub fn respond(
    cfg: &ServerConfig,
    state: &ClockState,
    request: &NtpPacket,
    receive_unix_ns: i64,
    transmit_unix_ns: i64,
) -> ServeDecision {
    // Before anything about our own clock: is this even a question? Answering a mode 4 or 5 makes
    // us a reflector — two such servers pointed at each other would answer each other forever, and
    // a spoofed source address turns every one of them into an amplifier.
    if request.mode != Mode::Client {
        return ServeDecision::Silent(SilentReason::NotARequest);
    }
    // We echo the client's version, so answering one we do not speak would put a number on the wire
    // whose meaning we have not implemented.
    if request.version == 0 || request.version > NTP_VERSION {
        return ServeDecision::Silent(SilentReason::UnsupportedVersion);
    }
    let last_update = match gate(cfg, state) {
        Ok(t) => t,
        Err(reason) => return ServeDecision::Silent(reason),
    };
    ServeDecision::Serve(NtpPacket {
        mode: Mode::Server,
        // Answer in the client's version and echo its poll — RFC 5905 §7.3. Substituting our own
        // would make the reply look like it belongs to a different association.
        version: request.version,
        poll: request.poll,
        // The client's own departure time, handed straight back. This is how it matches a reply to
        // its request; substitute anything else and every client discards every answer as bogus.
        origin_timestamp: request.transmit_timestamp,
        // The two ends of our processing, so the client can take it out of the delay it computes.
        receive_timestamp: NtpTimestamp::from_unix_ns(receive_unix_ns),
        transmit_timestamp: NtpTimestamp::from_unix_ns(transmit_unix_ns),
        ..base_packet(cfg, state, last_update)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const REF_UNIX_NS: i64 = 1_787_020_967 * 1_000_000_000;

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

    fn disciplined() -> ClockState {
        ClockState {
            last_update_unix_ns: Some(REF_UNIX_NS),
            holdover_ns: 0,
            frequency_locked: true,
            leap: LeapWarning::None,
        }
    }

    fn served(d: ServeDecision) -> NtpPacket {
        match d {
            ServeDecision::Serve(p) => p,
            ServeDecision::Silent(r) => panic!("expected a served packet, got Silent({r:?})"),
        }
    }

    #[test]
    fn a_disciplined_clock_serves_stratum_1_broadcast() {
        let p = served(broadcast(&cfg(), &disciplined(), REF_UNIX_NS + 500_000_000));
        assert_eq!(p.mode, Mode::Broadcast);
        assert_eq!(p.stratum, 1);
        assert_eq!(p.version, 4);
        assert_eq!(p.leap, LeapIndicator::NoWarning);
        assert_eq!(p.reference_id, *b"GPS\0");
        assert_eq!(p.poll, 4);
        assert_eq!(p.precision, -20);
    }

    #[test]
    fn root_delay_is_zero_for_a_primary_reference() {
        // Stratum 1 *is* the root: there is no upstream path to have delay to. The LAN delay to a
        // client belongs to the client's own measurement, not here.
        let p = served(broadcast(&cfg(), &disciplined(), REF_UNIX_NS));
        assert_eq!(p.root_delay, NtpShort::ZERO);
    }

    #[test]
    fn transmit_timestamp_is_the_scheduled_time_not_the_reference_time() {
        // The whole point of passing it in: it must describe when the frame leaves the wire.
        let tx = REF_UNIX_NS + 250_000_000;
        let p = served(broadcast(&cfg(), &disciplined(), tx));
        assert_eq!(p.transmit_timestamp, NtpTimestamp::from_unix_ns(tx));
        assert_eq!(
            p.reference_timestamp,
            NtpTimestamp::from_unix_ns(REF_UNIX_NS)
        );
    }

    #[test]
    fn origin_and_receive_timestamps_are_unused_in_broadcast() {
        let p = served(broadcast(&cfg(), &disciplined(), REF_UNIX_NS));
        assert_eq!(p.origin_timestamp, NtpTimestamp::ZERO);
        assert_eq!(p.receive_timestamp, NtpTimestamp::ZERO);
    }

    #[test]
    fn without_a_utc_epoch_we_say_nothing() {
        let st = ClockState {
            last_update_unix_ns: None,
            ..disciplined()
        };
        assert_eq!(
            broadcast(&cfg(), &st, REF_UNIX_NS),
            ServeDecision::Silent(SilentReason::NoEpoch)
        );
    }

    #[test]
    fn without_a_locked_frequency_we_say_nothing() {
        let st = ClockState {
            frequency_locked: false,
            ..disciplined()
        };
        assert_eq!(
            broadcast(&cfg(), &st, REF_UNIX_NS),
            ServeDecision::Silent(SilentReason::FrequencyUnlocked)
        );
    }

    #[test]
    fn a_short_holdover_still_serves_because_that_is_what_holdover_is_for() {
        let st = ClockState {
            holdover_ns: 60 * 1_000_000_000,
            ..disciplined()
        };
        let p = served(broadcast(&cfg(), &st, REF_UNIX_NS + 60_000_000_000));
        assert_eq!(p.stratum, 1);
    }

    #[test]
    fn holdover_past_the_limit_goes_silent() {
        let st = ClockState {
            holdover_ns: 3_601 * 1_000_000_000,
            ..disciplined()
        };
        assert_eq!(
            broadcast(&cfg(), &st, REF_UNIX_NS),
            ServeDecision::Silent(SilentReason::HoldoverExceeded)
        );
    }

    #[test]
    fn dispersion_starts_at_the_configured_floor() {
        assert_eq!(root_dispersion_ns(&cfg(), 0), 1_000_000);
    }

    #[test]
    fn dispersion_grows_with_the_frequency_bound_over_holdover() {
        // 100 ppb for 1000 s = 100e-9 * 1000 s = 100 µs, on top of the 1 ms floor.
        let ns = root_dispersion_ns(&cfg(), 1_000 * 1_000_000_000);
        assert_eq!(ns, 1_000_000 + 100_000);
    }

    #[test]
    fn dispersion_reaches_the_packet_as_a_saturating_short() {
        let st = ClockState {
            holdover_ns: 600 * 1_000_000_000,
            ..disciplined()
        };
        let p = served(broadcast(&cfg(), &st, REF_UNIX_NS));
        let expected = NtpShort::from_nanos(root_dispersion_ns(&cfg(), st.holdover_ns));
        assert_eq!(p.root_dispersion, expected);
        assert!(
            p.root_dispersion.to_bits() > 0,
            "a real holdover is not zero"
        );
    }

    #[test]
    fn absurd_holdover_does_not_overflow_the_dispersion_arithmetic() {
        // Guards the ns * ppb product: u64::MAX ns with a drift bound must not wrap into a small
        // dispersion, which would advertise a better clock than we have.
        let big = ServerConfig {
            max_holdover_ns: u64::MAX,
            // Absurd on purpose: at a realistic 100 ppb even u64::MAX ns of holdover only reaches
            // ~1844 s, which still fits an NtpShort and would not exercise the saturation.
            holdover_drift_ppb: 1_000_000_000,
            ..cfg()
        };
        let ns = root_dispersion_ns(&big, u64::MAX);
        assert!(
            ns >= big.base_dispersion_ns,
            "must not wrap below the floor"
        );
        assert_eq!(NtpShort::from_nanos(ns).to_bits(), u32::MAX);
    }

    // --- Unicast server mode (mode 3 -> 4) ---

    fn client_request() -> NtpPacket {
        NtpPacket {
            leap: LeapIndicator::NoWarning,
            version: 4,
            mode: Mode::Client,
            stratum: 0,
            poll: 6,
            precision: -20,
            root_delay: NtpShort::ZERO,
            root_dispersion: NtpShort::ZERO,
            reference_id: [0; 4],
            reference_timestamp: NtpTimestamp::ZERO,
            origin_timestamp: NtpTimestamp::ZERO,
            receive_timestamp: NtpTimestamp::ZERO,
            // The client's departure time, which it will look for coming back.
            transmit_timestamp: NtpTimestamp::from_unix_ns(REF_UNIX_NS - 3_000_000),
        }
    }

    #[test]
    fn a_reply_is_mode_server_at_stratum_1() {
        let p = served(respond(
            &cfg(),
            &disciplined(),
            &client_request(),
            REF_UNIX_NS,
            REF_UNIX_NS + 100_000,
        ));
        assert_eq!(p.mode, Mode::Server);
        assert_eq!(p.stratum, 1);
        assert_eq!(p.reference_id, *b"GPS\0");
        assert_eq!(p.root_delay, NtpShort::ZERO);
    }

    #[test]
    fn the_origin_timestamp_echoes_the_clients_transmit_timestamp() {
        // This is how a client matches a reply to its request; get it wrong and every client on the
        // network discards every answer as bogus.
        let req = client_request();
        let p = served(respond(
            &cfg(),
            &disciplined(),
            &req,
            REF_UNIX_NS,
            REF_UNIX_NS + 100_000,
        ));
        assert_eq!(p.origin_timestamp, req.transmit_timestamp);
    }

    #[test]
    fn receive_and_transmit_timestamps_are_the_two_moments_we_were_given() {
        // The client subtracts these to take our processing time out of its delay estimate, so they
        // must be the real arrival and departure rather than the same instant twice.
        let p = served(respond(
            &cfg(),
            &disciplined(),
            &client_request(),
            REF_UNIX_NS,
            REF_UNIX_NS + 250_000,
        ));
        assert_eq!(p.receive_timestamp, NtpTimestamp::from_unix_ns(REF_UNIX_NS));
        assert_eq!(
            p.transmit_timestamp,
            NtpTimestamp::from_unix_ns(REF_UNIX_NS + 250_000)
        );
        assert_ne!(p.receive_timestamp, p.transmit_timestamp);
    }

    #[test]
    fn the_reply_speaks_the_version_the_client_used() {
        // RFC 5905: a server answers in the client's version, not its own preference.
        let mut req = client_request();
        req.version = 3;
        let p = served(respond(
            &cfg(),
            &disciplined(),
            &req,
            REF_UNIX_NS,
            REF_UNIX_NS + 1,
        ));
        assert_eq!(p.version, 3);
    }

    #[test]
    fn the_reply_echoes_the_clients_poll_interval() {
        let req = client_request();
        let p = served(respond(
            &cfg(),
            &disciplined(),
            &req,
            REF_UNIX_NS,
            REF_UNIX_NS + 1,
        ));
        assert_eq!(p.poll, req.poll, "not our broadcast interval");
    }

    #[test]
    fn an_undisciplined_clock_answers_nothing() {
        let st = ClockState {
            frequency_locked: false,
            ..disciplined()
        };
        assert_eq!(
            respond(&cfg(), &st, &client_request(), REF_UNIX_NS, REF_UNIX_NS),
            ServeDecision::Silent(SilentReason::FrequencyUnlocked)
        );
    }

    #[test]
    fn a_reply_carries_the_same_holdover_dispersion_as_a_broadcast() {
        let st = ClockState {
            holdover_ns: 600 * 1_000_000_000,
            ..disciplined()
        };
        let p = served(respond(
            &cfg(),
            &st,
            &client_request(),
            REF_UNIX_NS,
            REF_UNIX_NS,
        ));
        assert_eq!(
            p.root_dispersion,
            NtpShort::from_nanos(root_dispersion_ns(&cfg(), st.holdover_ns))
        );
    }

    /// A secondary server following `upstream_stratum`, one round trip away, whose own distance to
    /// the root is `up_delay_ns` / `up_dispersion_ns`.
    fn secondary(
        upstream_stratum: u8,
        hop_ns: i64,
        up_delay_ns: u64,
        up_dispersion_ns: u64,
    ) -> ServerConfig {
        ServerConfig {
            source: Source::Upstream {
                address: [10, 0, 0, 1],
                stratum: upstream_stratum,
                root_delay: NtpShort::from_nanos(up_delay_ns),
                root_dispersion: NtpShort::from_nanos(up_dispersion_ns),
                delay_ns: hop_ns,
            },
            ..cfg()
        }
    }

    #[test]
    fn a_secondary_sits_one_stratum_below_what_it_follows() {
        let p = served(broadcast(
            &secondary(2, 0, 0, 0),
            &disciplined(),
            REF_UNIX_NS,
        ));
        assert_eq!(p.stratum, 3);
    }

    #[test]
    fn a_secondary_names_its_source_by_address_not_by_a_text_code() {
        // RFC 5905 §7.3: below stratum 1 the reference identifier is the upstream's IPv4 address.
        // Sending `GPS\0` from a secondary would claim hardware it does not have.
        let p = served(broadcast(
            &secondary(2, 0, 0, 0),
            &disciplined(),
            REF_UNIX_NS,
        ));
        assert_eq!(p.reference_id, [10, 0, 0, 1]);
    }

    #[test]
    fn root_delay_accumulates_the_hop_to_the_upstream() {
        // The field is the distance to the *root*, so our clients are one hop further out than we
        // are. Passing the upstream's figure through unchanged would understate every client's
        // uncertainty by exactly our own path.
        let up = 20_000_000;
        let hop = 6_000_000;
        let p = served(broadcast(
            &secondary(2, hop, up, 0),
            &disciplined(),
            REF_UNIX_NS,
        ));
        assert_eq!(p.root_delay, NtpShort::from_nanos(up + hop as u64));
    }

    #[test]
    fn root_dispersion_accumulates_on_top_of_the_upstreams() {
        let up = 30_000_000;
        let p = served(broadcast(
            &secondary(2, 0, 0, up),
            &disciplined(),
            REF_UNIX_NS,
        ));
        assert_eq!(
            p.root_dispersion,
            NtpShort::from_nanos(up + root_dispersion_ns(&secondary(2, 0, 0, up), 0))
        );
    }

    #[test]
    fn following_an_upstream_at_the_bottom_of_the_chain_goes_silent() {
        // Stratum 15 would make us 16, which on the wire *is* "unsynchronised". Serving it would
        // mean announcing we have no time while presenting ourselves as a source of it.
        assert_eq!(
            broadcast(&secondary(15, 0, 0, 0), &disciplined(), REF_UNIX_NS),
            ServeDecision::Silent(SilentReason::TooDeep)
        );
        // One stratum higher is still a usable server.
        let p = served(broadcast(
            &secondary(14, 0, 0, 0),
            &disciplined(),
            REF_UNIX_NS,
        ));
        assert_eq!(p.stratum, 15);
    }

    #[test]
    fn a_saturating_stratum_never_wraps_into_a_kiss_of_death() {
        // 255 + 1 would be 0, and stratum 0 means kiss-o'-death — a packet clients treat as an
        // instruction rather than as a time source.
        assert_eq!(
            broadcast(&secondary(255, 0, 0, 0), &disciplined(), REF_UNIX_NS),
            ServeDecision::Silent(SilentReason::TooDeep)
        );
    }

    #[test]
    fn an_announced_leap_second_reaches_the_packet() {
        // The point of carrying it: clients that never see the GNSS receiver still get to apply the
        // same leap at the same moment we do.
        for (warning, indicator) in [
            (LeapWarning::None, LeapIndicator::NoWarning),
            (LeapWarning::Insert, LeapIndicator::LastMinute61),
            (LeapWarning::Delete, LeapIndicator::LastMinute59),
        ] {
            let st = ClockState {
                leap: warning,
                ..disciplined()
            };
            let p = served(broadcast(&cfg(), &st, REF_UNIX_NS));
            assert_eq!(p.leap, indicator);
            let reply = served(respond(
                &cfg(),
                &st,
                &client_request(),
                REF_UNIX_NS,
                REF_UNIX_NS,
            ));
            assert_eq!(
                reply.leap, indicator,
                "both service modes say the same thing"
            );
        }
    }

    #[test]
    fn only_a_client_request_gets_an_answer() {
        // Answering a mode 4 or 5 makes this a reflector: two such servers pointed at each other
        // would answer each other forever, and a spoofed source address turns one into an
        // amplifier aimed at whoever it names.
        for mode in [
            Mode::Reserved,
            Mode::SymmetricActive,
            Mode::SymmetricPassive,
            Mode::Server,
            Mode::Broadcast,
            Mode::ControlMessage,
            Mode::Private,
        ] {
            let mut req = client_request();
            req.mode = mode;
            assert_eq!(
                respond(&cfg(), &disciplined(), &req, REF_UNIX_NS, REF_UNIX_NS),
                ServeDecision::Silent(SilentReason::NotARequest),
                "mode {mode:?} is not a question"
            );
        }
    }

    #[test]
    fn a_version_we_do_not_speak_gets_no_answer() {
        // The reply echoes the client's version, so answering an unknown one would put a number on
        // the wire whose meaning is not implemented here.
        for version in [0, NTP_VERSION + 1, 7] {
            let mut req = client_request();
            req.version = version;
            assert_eq!(
                respond(&cfg(), &disciplined(), &req, REF_UNIX_NS, REF_UNIX_NS),
                ServeDecision::Silent(SilentReason::UnsupportedVersion)
            );
        }
        // Older versions we do speak are still answered.
        for version in [1, 2, 3, NTP_VERSION] {
            let mut req = client_request();
            req.version = version;
            let p = served(respond(
                &cfg(),
                &disciplined(),
                &req,
                REF_UNIX_NS,
                REF_UNIX_NS,
            ));
            assert_eq!(p.version, version);
        }
    }

    #[test]
    fn what_is_wrong_with_the_request_is_decided_before_what_is_wrong_with_us() {
        // A caller uses these to tell "we cannot serve" from "that was not a request". Reporting an
        // undisciplined clock for a mode-5 packet would send them looking at the wrong subsystem.
        let mut req = client_request();
        req.mode = Mode::Broadcast;
        let undisciplined = ClockState {
            last_update_unix_ns: None,
            ..disciplined()
        };
        assert_eq!(
            respond(&cfg(), &undisciplined, &req, REF_UNIX_NS, REF_UNIX_NS),
            ServeDecision::Silent(SilentReason::NotARequest)
        );
    }

    #[test]
    fn may_serve_agrees_with_what_the_service_modes_decide() {
        // It exists so a caller can build a frame early and check eligibility late. If it could
        // disagree with the gate the modes use, that caller would send packets the policy refused.
        let cases = [
            disciplined(),
            ClockState {
                last_update_unix_ns: None,
                ..disciplined()
            },
            ClockState {
                frequency_locked: false,
                ..disciplined()
            },
            ClockState {
                holdover_ns: cfg().max_holdover_ns + 1,
                ..disciplined()
            },
        ];
        for state in cases {
            let by_broadcast = match broadcast(&cfg(), &state, REF_UNIX_NS) {
                ServeDecision::Serve(_) => Ok(()),
                ServeDecision::Silent(r) => Err(r),
            };
            assert_eq!(may_serve(&cfg(), &state), by_broadcast);
        }
    }
}
