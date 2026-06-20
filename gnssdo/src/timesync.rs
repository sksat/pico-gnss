//! Clock discipline via PPS (time sync).
//!
//! A GNSS 1PPS rising edge coincides with the UTC second boundary. By pairing the local timer
//! value at that instant (e.g. RP2040 TIMER, 1µs resolution) with the UTC second obtained from
//! NMEA, a µs-precision UTC epoch is maintained on the device.
//!
//! **Why on the device**: synchronizing on the host (over RTT/USB) adds the probe/USB round-trip
//! jitter (tens of ms) and destroys the PPS's inherent precision. The PPS-edge↔UTC-second pairing
//! must happen on an MCU that can timestamp the edge in µs.
//!
//! All of this is HAL-agnostic pure logic, so it is host-tested (`cargo test -p gnssdo`).

/// Days since 1970-01-01 (Howard Hinnant's algorithm, proleptic Gregorian).
pub const fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = (if y >= 0 { y } else { y - 399 }) / 400;
    let yoe = y - era * 400; // [0, 399]
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + d - 1; // [0, 365]
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy; // [0, 146096]
    era * 146097 + doe - 719468
}

/// UTC civil datetime → Unix seconds (leap seconds ignored).
pub const fn civil_to_unix(y: i64, mo: i64, d: i64, h: i64, mi: i64, s: i64) -> i64 {
    days_from_civil(y, mo, d) * 86400 + h * 3600 + mi * 60 + s
}

/// Extract (hour, min, sec) from an NMEA time field `hhmmss.sss`. The fractional seconds are
/// dropped (the second is an integer at the PPS boundary).
pub fn parse_hhmmss(field: &str) -> Option<(u8, u8, u8)> {
    let int_part = field.split('.').next().unwrap_or(field);
    if int_part.len() < 6 {
        return None;
    }
    let h: u8 = int_part.get(0..2)?.parse().ok()?;
    let mi: u8 = int_part.get(2..4)?.parse().ok()?;
    let s: u8 = int_part.get(4..6)?.parse().ok()?;
    if h > 23 || mi > 59 || s > 60 {
        return None;
    }
    Some((h, mi, s))
}

/// Extract (day, month, year) from an NMEA date field `ddmmyy` (RMC). The year assumes 20xx.
pub fn parse_ddmmyy(field: &str) -> Option<(u8, u8, u16)> {
    if field.len() < 6 {
        return None;
    }
    let d: u8 = field.get(0..2)?.parse().ok()?;
    let mo: u8 = field.get(2..4)?.parse().ok()?;
    let yy: u16 = field.get(4..6)?.parse().ok()?;
    if !(1..=31).contains(&d) || !(1..=12).contains(&mo) {
        return None;
    }
    Some((d, mo, 2000 + yy))
}

/// `((hour, minute, second), (day, month, year))` — the value returned by [`parse_rmc_time_date`].
pub type RmcTimeDate = ((u8, u8, u8), (u8, u8, u16));

/// Extract `((hour, min, sec), (day, month, year))` from an RMC sentence. `None` for non-RMC or
/// a parse failure.
///
/// The input is expected to be one assembler-framed NMEA sentence (`$` + 2-char talker, e.g.
/// `$GPRMC,...` / `$GNRMC,...`).
///
/// The default is the built-in parser. Enabling the `external-nmea` feature delegates to the
/// [`nmea`](https://docs.rs/nmea) crate. The backends differ:
/// - **checksum**: built-in = **not validated** / nmea = **validated** (mismatch → `None`).
/// - **year**: built-in = **fixed 20xx** (`2000+yy`) / nmea = **century pivot** (`yy=94` → 1994).
/// - **leap second `ss=60`**: built-in = **accepted** (civil_to_unix rolls into the next minute) /
///   nmea = **rejected** (`None`).
/// - **speed/size**: nmea is ~**17x slower** on the RP2040 and adds ~**+52 KB** of `.text`
///   (negligible at 1 Hz).
#[cfg(not(feature = "external-nmea"))]
pub fn parse_rmc_time_date(sentence: &str) -> Option<RmcTimeDate> {
    if sentence.get(3..6) != Some("RMC") {
        return None;
    }
    let time = sentence.split(',').nth(1).and_then(parse_hhmmss)?;
    let date = sentence.split(',').nth(9).and_then(parse_ddmmyy)?;
    Some((time, date))
}

/// Extract `((hour,min,sec),(day,month,year))` from RMC (nmea crate backend). See the default
/// version's docs. `nmea::parse_str` **validates the checksum** before extracting RMC
/// (mismatch → `None`).
#[cfg(feature = "external-nmea")]
pub fn parse_rmc_time_date(sentence: &str) -> Option<RmcTimeDate> {
    use chrono::{Datelike, Timelike};
    use nmea::ParseResult;
    // parse_str parses the sentence including checksum validation and returns RMC as ParseResult::RMC.
    let rmc = match nmea::parse_str(sentence).ok()? {
        ParseResult::RMC(rmc) => rmc,
        _ => return None,
    };
    let t = rmc.fix_time?;
    let d = rmc.fix_date?;
    Some((
        (t.hour() as u8, t.minute() as u8, t.second() as u8),
        (d.day() as u8, d.month() as u8, d.year() as u16),
    ))
}

/// An established sync point: a PPS edge's local time ↔ its UTC second.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyncPoint {
    /// Local timer value (µs) at which the PPS edge was timestamped.
    pub pps_local_us: u64,
    /// The UTC second (Unix seconds) that this PPS edge points to.
    pub unix_s: i64,
    /// Deviation of the latest PPS interval from the ideal 1 s (= local oscillator drift, µs).
    pub drift_us: i64,
}

/// How the second indicated by an NMEA time sentence (RMC/ZDA etc.) relates to the PPS edge it
/// follows.
///
/// The PPS edge marks the UTC second boundary itself, but whether the NMEA time sentence refers to
/// the **same / previous / next** second relative to that edge is receiver-dependent. Any offset is
/// always **within ±1 second** (it is an ordering question between the edge and the sentence; a
/// genuine ≥2 s offset does not occur in normal operation). Getting this wrong shifts the whole
/// established UTC epoch by 1 second — the biggest footgun in this library — so the three possible
/// values are enumerated and closed by the type (an arbitrary-second offset, which does not occur in
/// normal operation, is not accepted).
///
/// (`Debug`/`Default` is the minimum required by [`PpsTimeSync`]'s derive.)
#[derive(Debug, Default)]
pub enum PpsNmeaAssociation {
    /// The NMEA time sentence refers to the same second as the edge (most receivers). Default.
    #[default]
    SameSecond,
    /// The NMEA time sentence refers to the previous second (the edge is 1 s after the NMEA time).
    NmeaIsPreviousSecond,
    /// The NMEA time sentence refers to the next second (the edge is 1 s before the NMEA time).
    NmeaIsNextSecond,
}

impl PpsNmeaAssociation {
    /// Edge UTC second = (NMEA second) + this correction (±1 or 0).
    const fn edge_offset_seconds(&self) -> i64 {
        match self {
            Self::SameSecond => 0,
            Self::NmeaIsPreviousSecond => 1,
            Self::NmeaIsNextSecond => -1,
        }
    }
}

/// State machine that disciplines the clock by pairing PPS edges with NMEA time.
#[derive(Debug, Default)]
pub struct PpsTimeSync {
    association: PpsNmeaAssociation,
    last_date: Option<(u16, u8, u8)>, // (year, month, day)
    pending_pps_us: Option<u64>,      // most recent PPS edge not yet paired with a UTC second
    last_pps_us: Option<u64>,         // previous PPS edge, for the drift calculation
    epoch_local_us: Option<u64>,      // established epoch: local reference
    epoch_unix_s: Option<i64>,        // established epoch: UTC second
    last_drift_us: i64,
}

impl PpsTimeSync {
    /// Create with the default ([`PpsNmeaAssociation::SameSecond`]).
    pub const fn new() -> Self {
        Self::with_association(PpsNmeaAssociation::SameSecond)
    }

    /// Create with a given PPS↔NMEA second association. `const fn` so it can be used in `static` init.
    pub const fn with_association(association: PpsNmeaAssociation) -> Self {
        Self {
            association,
            last_date: None,
            pending_pps_us: None,
            last_pps_us: None,
            epoch_local_us: None,
            epoch_unix_s: None,
            last_drift_us: 0,
        }
    }

    /// Record a PPS rising edge at local time `local_us`.
    /// If a previous edge exists, returns the drift (interval − 1 s, µs).
    pub fn on_pps(&mut self, local_us: u64) -> Option<i64> {
        let drift = self
            .last_pps_us
            .map(|prev| local_us as i64 - prev as i64 - 1_000_000);
        if let Some(d) = drift {
            self.last_drift_us = d;
        }
        self.last_pps_us = Some(local_us);
        self.pending_pps_us = Some(local_us);
        drift
    }

    /// Update the date from RMC/ZDA.
    pub fn set_date(&mut self, year: u16, month: u8, day: u8) {
        self.last_date = Some((year, month, day));
    }

    /// Take the NMEA time (h,mi,s), pair it with the most recent PPS edge, and establish a sync
    /// point. `None` if the date is unknown or no PPS has been seen.
    pub fn on_time(&mut self, h: u8, mi: u8, s: u8) -> Option<SyncPoint> {
        let (y, mo, d) = self.last_date?;
        let pps = self.pending_pps_us?;
        // UTC second of the PPS edge = NMEA second + receiver-dependent correction (absorbs ±1 s).
        let unix_s = civil_to_unix(y as i64, mo as i64, d as i64, h as i64, mi as i64, s as i64)
            + self.association.edge_offset_seconds();
        self.epoch_local_us = Some(pps);
        self.epoch_unix_s = Some(unix_s);
        self.pending_pps_us = None;
        Some(SyncPoint {
            pps_local_us: pps,
            unix_s,
            drift_us: self.last_drift_us,
        })
    }

    /// Whether sync has been established.
    pub fn is_locked(&self) -> bool {
        self.epoch_local_us.is_some() && self.epoch_unix_s.is_some()
    }

    /// Disciplined UTC (Unix µs) for an arbitrary local timer value. `None` if not synced.
    pub fn now_unix_micros(&self, local_us: u64) -> Option<i64> {
        let el = self.epoch_local_us?;
        let eu = self.epoch_unix_s?;
        Some(eu * 1_000_000 + (local_us as i64 - el as i64))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn civil_to_unix_known_anchors() {
        assert_eq!(civil_to_unix(1970, 1, 1, 0, 0, 0), 0);
        assert_eq!(civil_to_unix(2000, 1, 1, 0, 0, 0), 946_684_800);
        // 2026-06-07T17:06:59Z (a measured fix; verified with `date -u -d ... +%s`)
        assert_eq!(civil_to_unix(2026, 6, 7, 17, 6, 59), 1_780_852_019);
    }

    #[test]
    fn parse_time_ok() {
        assert_eq!(parse_hhmmss("170658.000"), Some((17, 6, 58)));
        assert_eq!(parse_hhmmss("000000.00"), Some((0, 0, 0)));
        assert_eq!(parse_hhmmss("235959"), Some((23, 59, 59)));
    }

    #[test]
    fn parse_time_rejects_garbage() {
        assert_eq!(parse_hhmmss(""), None);
        assert_eq!(parse_hhmmss("12345"), None); // too short
        assert_eq!(parse_hhmmss("99xxss"), None);
        assert_eq!(parse_hhmmss("250000"), None); // hour > 23
    }

    #[test]
    fn parse_rmc_time_date_extracts_time() {
        let s = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
        let r = parse_rmc_time_date(s).unwrap();
        assert_eq!(r.0, (12, 35, 19)); // the time matches on both backends
        // The year interpretation differs: built-in = fixed 20xx (2094), nmea = century pivot (1994).
        #[cfg(not(feature = "external-nmea"))]
        assert_eq!(r.1, (23, 3, 2094));
        #[cfg(feature = "external-nmea")]
        assert_eq!(r.1, (23, 3, 1994));
    }

    #[test]
    fn parse_rmc_time_date_rejects_non_rmc() {
        let s = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        assert_eq!(parse_rmc_time_date(s), None);
    }

    #[test]
    fn parse_rmc_time_date_checksum_behavior() {
        // RMC with a tampered checksum (correct is *6A, wrong is *00).
        let bad = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*00";
        // The built-in parser doesn't validate the checksum, so it passes; nmea's parse_str rejects it.
        #[cfg(not(feature = "external-nmea"))]
        assert!(parse_rmc_time_date(bad).is_some());
        #[cfg(feature = "external-nmea")]
        assert!(parse_rmc_time_date(bad).is_none());
    }

    #[test]
    fn parse_date_ok() {
        assert_eq!(parse_ddmmyy("070626"), Some((7, 6, 2026)));
        assert_eq!(parse_ddmmyy("311299"), Some((31, 12, 2099)));
    }

    #[test]
    fn parse_date_rejects_garbage() {
        assert_eq!(parse_ddmmyy("00xx26"), None);
        assert_eq!(parse_ddmmyy("001326"), None); // month 13
        assert_eq!(parse_ddmmyy("12"), None);
    }

    #[test]
    fn drift_is_none_then_interval_error() {
        let mut ts = PpsTimeSync::new();
        assert_eq!(ts.on_pps(1_000_000), None); // first edge has no previous
        assert_eq!(ts.on_pps(2_000_050), Some(50)); // +50µs drift
        assert_eq!(ts.on_pps(2_999_940), Some(-110)); // -110µs
    }

    #[test]
    fn sync_requires_both_date_and_pps() {
        let mut ts = PpsTimeSync::new();
        // Neither PPS-only nor time-only establishes sync.
        assert!(ts.on_time(17, 6, 58).is_none());
        ts.on_pps(1_000_000);
        assert!(ts.on_time(17, 6, 58).is_none()); // date not yet set
        ts.set_date(2026, 6, 7);
        let sp = ts.on_time(17, 6, 58).unwrap();
        assert_eq!(sp.pps_local_us, 1_000_000);
        assert_eq!(sp.unix_s, civil_to_unix(2026, 6, 7, 17, 6, 58));
        assert!(ts.is_locked());
    }

    #[test]
    fn now_micros_interpolates_from_epoch() {
        let mut ts = PpsTimeSync::new();
        ts.set_date(2026, 6, 7);
        ts.on_pps(1_000_000);
        let sp = ts.on_time(17, 6, 58).unwrap();
        let base = sp.unix_s * 1_000_000;
        // the epoch itself
        assert_eq!(ts.now_unix_micros(1_000_000), Some(base));
        // 0.5 s later
        assert_eq!(ts.now_unix_micros(1_500_000), Some(base + 500_000));
        // before the epoch (a local value earlier than the PPS)
        assert_eq!(ts.now_unix_micros(999_000), Some(base - 1_000));
    }

    #[test]
    fn association_nmea_next_second_shifts_epoch_back() {
        // The receiver sends the "next" second early: the edge's UTC second is NMEA second − 1.
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsNextSecond);
        ts.set_date(2026, 6, 7);
        ts.on_pps(1_000_000);
        let sp = ts.on_time(17, 6, 58).unwrap();
        // The default (SameSecond) would be 17:06:58, but NmeaIsNextSecond maps it to 17:06:57.
        assert_eq!(sp.unix_s, civil_to_unix(2026, 6, 7, 17, 6, 57));
        // `now` reflects the same epoch shift.
        assert_eq!(
            ts.now_unix_micros(1_000_000),
            Some(civil_to_unix(2026, 6, 7, 17, 6, 57) * 1_000_000)
        );
    }

    #[test]
    fn association_nmea_previous_second_shifts_epoch_forward() {
        // The receiver reports the "previous" second: the edge's UTC second is NMEA second + 1.
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsPreviousSecond);
        ts.set_date(2026, 6, 7);
        ts.on_pps(1_000_000);
        let sp = ts.on_time(17, 6, 58).unwrap();
        assert_eq!(sp.unix_s, civil_to_unix(2026, 6, 7, 17, 6, 59));
    }

    #[test]
    fn association_handles_day_rollover() {
        // 00:00:00 with NmeaIsNextSecond(-1) → previous day 23:59:59 (Unix-second math handles the rollover).
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsNextSecond);
        ts.set_date(2026, 6, 7);
        ts.on_pps(5_000_000);
        let sp = ts.on_time(0, 0, 0).unwrap();
        assert_eq!(sp.unix_s, civil_to_unix(2026, 6, 6, 23, 59, 59));
    }

    #[test]
    fn association_handles_year_rollover() {
        // 2026-12-31 23:59:59 with NmeaIsPreviousSecond(+1) → 2027-01-01 00:00:00 (year rollover).
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsPreviousSecond);
        ts.set_date(2026, 12, 31);
        ts.on_pps(1_000_000);
        let sp = ts.on_time(23, 59, 59).unwrap();
        assert_eq!(sp.unix_s, civil_to_unix(2027, 1, 1, 0, 0, 0));
    }

    #[test]
    fn association_offset_does_not_accumulate() {
        // Across consecutive syncs the correction does not accumulate (not +1 per second; it applies
        // equally to both epochs).
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsNextSecond);
        ts.set_date(2026, 6, 7);
        ts.on_pps(1_000_000);
        let s1 = ts.on_time(17, 6, 58).unwrap();
        ts.on_pps(2_000_000);
        let s2 = ts.on_time(17, 6, 59).unwrap();
        assert_eq!(s2.unix_s - s1.unix_s, 1);
        assert_eq!(s1.unix_s, civil_to_unix(2026, 6, 7, 17, 6, 57));
        assert_eq!(s2.unix_s, civil_to_unix(2026, 6, 7, 17, 6, 58));
    }

    #[test]
    fn sync_advances_each_second() {
        let mut ts = PpsTimeSync::new();
        ts.set_date(2026, 6, 7);
        ts.on_pps(1_000_000);
        let s1 = ts.on_time(17, 6, 58).unwrap();
        ts.on_pps(2_000_000);
        let s2 = ts.on_time(17, 6, 59).unwrap();
        assert_eq!(s2.unix_s - s1.unix_s, 1);
        assert_eq!(s2.pps_local_us, 2_000_000);
        assert_eq!(s2.drift_us, 0); // exactly a 1 s interval
    }
}
