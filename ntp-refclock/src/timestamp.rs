//! NTP's two fixed-point time formats (RFC 5905 §6) and their Unix-nanosecond conversions.
//!
//! Both are unsigned binary fixed-point, big-endian on the wire:
//!
//! - [`NtpTimestamp`] — 32.32: seconds since the 1900 prime epoch, plus a 2⁻³² s (≈233 ps)
//!   fraction. Used for the four timestamps in a packet.
//! - [`NtpShort`] — 16.16: seconds plus a 2⁻¹⁶ s (≈15.3 µs) fraction. Used for root delay and root
//!   dispersion.
//!
//! Everything here is integer-only; the conversions round to nearest so a whole number of
//! nanoseconds survives a round trip exactly (2⁻³² s is finer than 1 ns).

/// Seconds between the NTP prime epoch (1900-01-01) and the Unix epoch (1970-01-01), including the
/// 17 leap days in between.
pub const NTP_UNIX_OFFSET_SECS: u64 = 2_208_988_800;

const NANOS_PER_SEC: i64 = 1_000_000_000;
/// Seconds in one NTP era — the 32-bit seconds field wraps every 2³² s (≈136 years).
const ERA_SECS: i64 = 1 << 32;

/// NTP 64-bit timestamp (RFC 5905 §6): 32 bits of seconds since 1900-01-01, 32 bits of fraction.
///
/// The era is **not** on the wire, so a bare timestamp is ambiguous every ≈136 years. Convert with
/// [`to_unix_ns_near`](Self::to_unix_ns_near) when decoding, which disambiguates against a roughly
/// known local time.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct NtpTimestamp {
    secs: u32,
    frac: u32,
}

impl NtpTimestamp {
    /// The all-zero timestamp, which RFC 5905 uses to mean "unspecified".
    pub const ZERO: Self = Self { secs: 0, frac: 0 };

    /// From the 64-bit wire value (seconds in the high half, fraction in the low half).
    pub const fn from_bits(bits: u64) -> Self {
        Self {
            secs: (bits >> 32) as u32,
            frac: bits as u32,
        }
    }

    /// The 64-bit wire value.
    pub const fn to_bits(self) -> u64 {
        ((self.secs as u64) << 32) | self.frac as u64
    }

    /// Seconds field (since 1900-01-01, wrapped into the current era).
    pub const fn seconds(self) -> u32 {
        self.secs
    }

    /// Fraction field, in units of 2⁻³² s.
    pub const fn fraction(self) -> u32 {
        self.frac
    }

    /// Which NTP era a Unix instant falls in (0 = 1900-01-01 … 2036-02-07).
    pub const fn era_of_unix_ns(unix_ns: i64) -> i64 {
        let ntp_full = unix_ns.div_euclid(NANOS_PER_SEC) + NTP_UNIX_OFFSET_SECS as i64;
        ntp_full.div_euclid(ERA_SECS)
    }

    /// Disciplined UTC (Unix ns) → wire timestamp. The era is dropped (it is not transmitted);
    /// negative Unix times floor toward the past, so the fraction is always non-negative.
    pub const fn from_unix_ns(unix_ns: i64) -> Self {
        let secs_floor = unix_ns.div_euclid(NANOS_PER_SEC);
        let nanos = unix_ns.rem_euclid(NANOS_PER_SEC) as u64;
        let ntp_full = secs_floor + NTP_UNIX_OFFSET_SECS as i64;
        // Round to nearest 2^-32 s. `nanos < 1e9`, so `nanos << 32` stays well inside u64.
        let frac = ((nanos << 32) + (NANOS_PER_SEC as u64) / 2) / NANOS_PER_SEC as u64;
        Self {
            secs: ntp_full.rem_euclid(ERA_SECS) as u32,
            frac: frac as u32,
        }
    }

    /// Wire timestamp → Unix ns, given the era it belongs to. Saturates rather than overflowing for
    /// eras far beyond what `i64` nanoseconds can express (year ≈2262).
    pub const fn to_unix_ns_in_era(self, era: i64) -> i64 {
        let ntp_full = era * ERA_SECS + self.secs as i64;
        let unix_secs = ntp_full - NTP_UNIX_OFFSET_SECS as i64;
        // Round to nearest ns. `frac * 1e9` stays inside u64 for any u32 fraction.
        let nanos = ((self.frac as u64) * NANOS_PER_SEC as u64 + (1 << 31)) >> 32;
        unix_secs
            .saturating_mul(NANOS_PER_SEC)
            .saturating_add(nanos as i64)
    }

    /// Wire timestamp → Unix ns, resolving the era to whichever one puts the result closest to
    /// `reference_unix_ns`. This is what a receiver needs: 32 bits of seconds alone cannot say
    /// which 136-year era they came from.
    pub fn to_unix_ns_near(self, reference_unix_ns: i64) -> i64 {
        let ref_ntp_full =
            reference_unix_ns.div_euclid(NANOS_PER_SEC) + NTP_UNIX_OFFSET_SECS as i64;
        let ref_era = ref_ntp_full.div_euclid(ERA_SECS);
        // Compare in *seconds*, where no era overflows i64, then convert only the winner.
        let mut best_era = ref_era;
        let mut best_diff = i64::MAX;
        let mut era = ref_era - 1;
        while era <= ref_era + 1 {
            let cand = era * ERA_SECS + self.secs as i64;
            let diff = (cand - ref_ntp_full).abs();
            if diff < best_diff {
                best_diff = diff;
                best_era = era;
            }
            era += 1;
        }
        self.to_unix_ns_in_era(best_era)
    }
}

/// NTP 32-bit "short" format (RFC 5905 §6): 16 bits of seconds, 16 bits of fraction (2⁻¹⁶ s).
/// Root delay and root dispersion use it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct NtpShort(u32);

impl NtpShort {
    /// The all-zero value.
    pub const ZERO: Self = Self(0);

    /// From the 32-bit wire value.
    pub const fn from_bits(bits: u32) -> Self {
        Self(bits)
    }

    /// The 32-bit wire value.
    pub const fn to_bits(self) -> u32 {
        self.0
    }

    /// Nanoseconds → 16.16, rounding to nearest and **saturating** at the 65536 s ceiling.
    ///
    /// Saturation matters: root dispersion grows without bound during a long holdover, and wrapping
    /// would advertise a *better* clock than we actually have.
    pub const fn from_nanos(nanos: u64) -> Self {
        let secs = nanos / NANOS_PER_SEC as u64;
        if secs > u16::MAX as u64 {
            return Self(u32::MAX);
        }
        let rem = nanos % NANOS_PER_SEC as u64;
        let frac = (rem * 65536 + (NANOS_PER_SEC as u64) / 2) / NANOS_PER_SEC as u64;
        let bits = (secs << 16) + frac;
        if bits > u32::MAX as u64 {
            Self(u32::MAX)
        } else {
            Self(bits as u32)
        }
    }

    /// 16.16 → nanoseconds, rounding to nearest.
    pub const fn to_nanos(self) -> u64 {
        let secs = (self.0 >> 16) as u64;
        let frac = (self.0 & 0xFFFF) as u64;
        secs * NANOS_PER_SEC as u64 + (frac * NANOS_PER_SEC as u64 + 32768) / 65536
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- NtpTimestamp: 32.32 seconds since 1900-01-01T00:00:00Z ---

    #[test]
    fn unix_epoch_is_the_ntp_prime_epoch_offset() {
        // RFC 5905 §6: the NTP prime epoch is 1900-01-01; Unix's is 1970-01-01. The gap is 70 years
        // including 17 leap days = 2_208_988_800 s.
        assert_eq!(NTP_UNIX_OFFSET_SECS, 2_208_988_800);
        let t = NtpTimestamp::from_unix_ns(0);
        assert_eq!(t.seconds(), 2_208_988_800);
        assert_eq!(t.fraction(), 0);
    }

    #[test]
    fn known_anchor_matches_hand_computed_value() {
        // 2026-08-18T02:42:47Z = Unix 1_787_020_967 (a UTC second taken from a real GNSS fix).
        let t = NtpTimestamp::from_unix_ns(1_787_020_967 * 1_000_000_000);
        assert_eq!(t.seconds(), 1_787_020_967 + 2_208_988_800);
        assert_eq!(t.fraction(), 0);
    }

    #[test]
    fn half_a_second_is_the_top_fraction_bit() {
        let t = NtpTimestamp::from_unix_ns(500_000_000);
        assert_eq!(t.seconds(), 2_208_988_800);
        assert_eq!(t.fraction(), 0x8000_0000);
    }

    #[test]
    fn fraction_resolution_is_finer_than_a_nanosecond() {
        // 2^-32 s ≈ 232.8 ps, so every whole nanosecond is representable and distinct.
        let a = NtpTimestamp::from_unix_ns(1);
        let b = NtpTimestamp::from_unix_ns(2);
        assert_ne!(a.fraction(), b.fraction());
        assert!(a.fraction() > 0);
    }

    #[test]
    fn nanosecond_round_trip_is_exact_within_a_second() {
        // Distinct ns map to distinct fractions (see above), so rounding back must recover them.
        for ns in [0i64, 1, 999, 1_000_000, 123_456_789, 999_999_998, 999_999_999] {
            let t = NtpTimestamp::from_unix_ns(ns);
            assert_eq!(t.to_unix_ns_in_era(0), ns, "ns={ns}");
        }
    }

    #[test]
    fn negative_unix_times_floor_toward_the_past() {
        // -1 ns is 1900-epoch second 2_208_988_799 plus fraction 1-1e-9, not second 2_208_988_800
        // with a negative fraction. Euclidean division, not truncation.
        let t = NtpTimestamp::from_unix_ns(-1);
        assert_eq!(t.seconds(), 2_208_988_799);
        assert_eq!(t.to_unix_ns_in_era(0), -1);
    }

    #[test]
    fn era_wraps_at_2036_02_07() {
        // NTP seconds are 32-bit and wrap after 2^32 s: 2036-02-07T06:28:16Z = Unix 2_085_978_496.
        const ERA1_START_UNIX: i64 = 2_085_978_496;
        assert_eq!(NtpTimestamp::era_of_unix_ns(0), 0);
        assert_eq!(
            NtpTimestamp::era_of_unix_ns((ERA1_START_UNIX - 1) * 1_000_000_000),
            0
        );
        assert_eq!(
            NtpTimestamp::era_of_unix_ns(ERA1_START_UNIX * 1_000_000_000),
            1
        );
        // The wire value restarts from zero in era 1 — the era is *not* on the wire.
        let t = NtpTimestamp::from_unix_ns(ERA1_START_UNIX * 1_000_000_000);
        assert_eq!(t.seconds(), 0);
        assert_eq!(t.to_unix_ns_in_era(1), ERA1_START_UNIX * 1_000_000_000);
    }

    #[test]
    fn decoding_picks_the_era_nearest_a_reference_instant() {
        // A receiver only sees 32 bits, so it must disambiguate the era against a rough local clock.
        const NOW: i64 = 1_787_020_967; // 2026-08-18, era 0
        const ONE_ERA_LATER: i64 = NOW + 4_294_967_296; // same 32 bits, ~2162
        let wire = NtpTimestamp::from_unix_ns(NOW * 1_000_000_000);
        assert_eq!(wire.to_unix_ns_near(NOW * 1_000_000_000), NOW * 1_000_000_000);
        // Identical wire bits, read from a vantage point one era later, resolve to that era.
        assert_eq!(
            wire.to_unix_ns_near(ONE_ERA_LATER * 1_000_000_000),
            ONE_ERA_LATER * 1_000_000_000
        );
    }

    #[test]
    fn raw_wire_encoding_is_seconds_then_fraction_big_endian() {
        let t = NtpTimestamp::from_unix_ns(500_000_000);
        assert_eq!(t.to_bits(), 0x83AA_7E80_8000_0000);
        assert_eq!(NtpTimestamp::from_bits(t.to_bits()), t);
    }

    // --- NtpShort: 16.16 seconds, used by root delay / root dispersion ---

    #[test]
    fn short_one_second_is_the_integer_bit() {
        assert_eq!(NtpShort::from_nanos(1_000_000_000).to_bits(), 0x0001_0000);
        assert_eq!(NtpShort::from_nanos(0).to_bits(), 0);
    }

    #[test]
    fn short_resolution_is_about_15_microseconds() {
        // 2^-16 s = 15258.789 ns. A value below half of that rounds to zero.
        assert_eq!(NtpShort::from_nanos(7_000).to_bits(), 0);
        assert_eq!(NtpShort::from_nanos(15_259).to_bits(), 1);
        assert_eq!(NtpShort::from_nanos(15_259).to_nanos(), 15_259);
    }

    #[test]
    fn short_saturates_instead_of_wrapping() {
        // Root dispersion grows without bound during a long holdover; it must clamp, not wrap to a
        // small value (which would advertise a *better* clock than we have).
        let huge = NtpShort::from_nanos(u64::MAX);
        assert_eq!(huge.to_bits(), u32::MAX);
        assert_eq!(NtpShort::from_nanos(65_536 * 1_000_000_000).to_bits(), u32::MAX);
        // Just under the ceiling still encodes normally.
        assert!(NtpShort::from_nanos(65_535 * 1_000_000_000).to_bits() < u32::MAX);
    }

    #[test]
    fn short_round_trip_is_exact_at_representable_values() {
        for bits in [0u32, 1, 0x0001_0000, 0x1234_5678, 0xFFFF_0000] {
            let s = NtpShort::from_bits(bits);
            assert_eq!(NtpShort::from_nanos(s.to_nanos()).to_bits(), bits, "bits={bits:#x}");
        }
    }
}
