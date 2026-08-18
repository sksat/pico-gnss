//! Stratum-1 server policy: deciding *whether* we may serve the time, and what to claim about it.
//!
//! The wire format ([`crate::packet`]) is neutral about stratum. This module is where being a
//! **reference clock** shows up: refusing to transmit while undisciplined, growing root dispersion
//! through a holdover, and pinning stratum 1 with a source identifier.
//!
//! It takes the clock's state as plain integers, so it is testable on the host without a GNSS
//! receiver anywhere nearby.

use crate::packet::{LeapIndicator, Mode, NtpPacket};
use crate::timestamp::{NtpShort, NtpTimestamp};

/// Static description of this reference clock.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ServerConfig {
    /// log2 seconds of the resolution at which we can actually *timestamp* a transmission.
    ///
    /// This is not the oscillator's precision. Claiming better than we can timestamp corrupts a
    /// client's source selection, so it must come from measurement of the transmit path.
    pub precision: i8,
    /// log2 seconds between broadcasts.
    pub poll: i8,
    /// Four-character source code (RFC 5905 §7.3), e.g. `GPS\0`.
    pub reference_id: [u8; 4],
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
pub fn broadcast(cfg: &ServerConfig, state: &ClockState, transmit_unix_ns: i64) -> ServeDecision {
    // Order matters: not knowing the time at all is a different (and prior) failure to knowing it
    // but being unable to bound how fast we are losing it.
    let Some(last_update) = state.last_update_unix_ns else {
        return ServeDecision::Silent(SilentReason::NoEpoch);
    };
    if !state.frequency_locked {
        return ServeDecision::Silent(SilentReason::FrequencyUnlocked);
    }
    if state.holdover_ns > cfg.max_holdover_ns {
        return ServeDecision::Silent(SilentReason::HoldoverExceeded);
    }

    ServeDecision::Serve(NtpPacket {
        leap: LeapIndicator::NoWarning,
        version: NTP_VERSION,
        mode: Mode::Broadcast,
        stratum: 1,
        poll: cfg.poll,
        precision: cfg.precision,
        // Stratum 1 *is* the root: there is no upstream path to have delay to.
        root_delay: NtpShort::ZERO,
        root_dispersion: NtpShort::from_nanos(root_dispersion_ns(cfg, state.holdover_ns)),
        reference_id: cfg.reference_id,
        reference_timestamp: NtpTimestamp::from_unix_ns(last_update),
        // A broadcast answers no request, so there is no origin to echo and no arrival to report.
        origin_timestamp: NtpTimestamp::ZERO,
        receive_timestamp: NtpTimestamp::ZERO,
        transmit_timestamp: NtpTimestamp::from_unix_ns(transmit_unix_ns),
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
            reference_id: *b"GPS\0",
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
}
