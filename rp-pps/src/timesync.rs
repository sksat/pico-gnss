//! Pair a PPS edge with NMEA time to produce a UTC epoch.
//!
//! A GNSS 1PPS rising edge coincides with the UTC second boundary. [`PpsTimeSync`] timestamps the
//! edge on the device's two timebases (capture + query, see [`SyncEpoch`]) and pairs it with the
//! UTC second obtained from NMEA, yielding a [`SyncEpoch`] to feed the discipline core
//! ([`gnssdo`](https://docs.rs/gnssdo)). This crate owns only the *pairing* (and the ±1 s
//! receiver-dependent association); the disciplined clock / holdover / "now" queries are `gnssdo`.
//!
//! **Why on the device**: pairing on the host (over RTT/USB) adds the probe/USB round-trip jitter
//! (tens of ms) and destroys the PPS's inherent precision; the edge must be timestamped on the MCU.
//!
//! All of this is HAL-agnostic pure logic, so it is host-tested (`cargo test -p rp-pps`).

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

/// Extract `((hour,min,sec),(day,month,year))` from a **ZDA** sentence.
///
/// `$GPZDA,hhmmss.sss,dd,mm,yyyy,zh,zm*CS` — time, then day, month and a **four-digit** year, so
/// unlike RMC there is no two-digit year to guess a century for.
///
/// # Why this sentence in particular
///
/// ZDA is the one NMEA sentence defined *against the timing pulse*. The MT3333 platform NMEA
/// specification lists it as the "PPS timing message (synchronized to PPS)" and says it "outputs the
/// time associated with the current 1PPS pulse … and tells the time of the pulse that just
/// occurred". RMC carries a time too, but nothing in any specification ties it to the pulse — it is
/// a navigation sentence that happens to contain a clock.
///
/// Pairing on a sentence with no defined relationship to the edge is how a receiver ends up
/// labelling the pulse with the wrong second.
pub fn parse_zda_time_date(sentence: &str) -> Option<RmcTimeDate> {
    if sentence.get(3..6) != Some("ZDA") {
        return None;
    }
    let mut f = sentence.split(',');
    let time = f.nth(1).and_then(parse_hhmmss)?;
    let day: u8 = f.next()?.parse().ok()?;
    let month: u8 = f.next()?.parse().ok()?;
    let year_field = f.next()?;
    // ZDA's year is four digits by definition; anything else is not the field we think it is.
    if year_field.len() != 4 {
        return None;
    }
    let year: u16 = year_field.parse().ok()?;
    if year < 1980 {
        return None;
    }
    // Check the real length of the month, leap years included. A bare `day <= 31` would accept
    // 31 February, and `civil_to_unix` normalises that into 3 March without complaint — quietly
    // establishing a UTC epoch two days off. Rejecting is the only safe answer: a receiver that
    // emits an impossible date is not one whose time should be believed.
    if !(1..=12).contains(&month) || day < 1 || day > days_in_month(month, year) {
        return None;
    }
    Some((time, (day, month, year)))
}

/// Length of a Gregorian month, leap years included. 0 for an invalid month.
const fn days_in_month(month: u8, year: u16) -> u8 {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 => {
            let leap =
                (year.is_multiple_of(4) && !year.is_multiple_of(100)) || year.is_multiple_of(400);
            if leap { 29 } else { 28 }
        }
        _ => 0,
    }
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

/// A UTC epoch from pairing a PPS edge with its UTC instant: the edge timestamped on both device
/// timebases (integer ns) and the Unix-nanosecond instant it marks. Feed it straight to
/// [`gnssdo`](https://docs.rs/gnssdo)'s `DisciplinedClock::update_epoch` / `Gnssdo::on_utc`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyncEpoch {
    /// High-resolution capture-timebase value (ns) at the PPS edge (e.g. RP2040 PIO capture).
    pub capture_ns: u64,
    /// Continuously-readable query-timebase value (ns) at the same edge (e.g. embassy `Instant`).
    pub query_ns: u64,
    /// The UTC instant (Unix nanoseconds) that this PPS edge marks.
    pub unix_ns: i64,
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

/// Pairs PPS edges with NMEA time to produce a UTC [`SyncEpoch`] for the discipline core.
///
/// It does only the *pairing* (including the ±1 s [`PpsNmeaAssociation`] correction). The
/// disciplined clock, holdover and "now" queries are [`gnssdo`](https://docs.rs/gnssdo)'s job — feed
/// the [`SyncEpoch`] this produces to `DisciplinedClock::update_epoch` / `Gnssdo::on_utc`.
#[derive(Debug, Default)]
pub struct PpsTimeSync {
    association: PpsNmeaAssociation,
    last_date: Option<(u16, u8, u8)>, // (year, month, day) from RMC/ZDA
    pending: Option<(u64, u64)>,      // (capture_ns, query_ns) of the most recent un-paired edge
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
            pending: None,
        }
    }

    /// Record a PPS rising edge, timestamped on the capture and query timebases (integer ns, see
    /// [`SyncEpoch`]). The edge is held until the next [`on_time`](Self::on_time) pairs it.
    pub fn on_pps_edge(&mut self, capture_ns: u64, query_ns: u64) {
        self.pending = Some((capture_ns, query_ns));
    }

    /// Update the date from an RMC/ZDA sentence.
    pub fn set_date(&mut self, year: u16, month: u8, day: u8) {
        self.last_date = Some((year, month, day));
    }

    /// Pair the NMEA time `(h, mi, s)` with the most recent un-paired PPS edge and return the UTC
    /// [`SyncEpoch`]. `None` if the date is unknown or no fresh edge is pending.
    ///
    /// Pairing **consumes** the edge: a second call with no new [`on_pps_edge`](Self::on_pps_edge)
    /// in between returns `None`, so a stale edge is never paired to two different seconds (which
    /// would inject a ±integer-second epoch error while PPS is missing).
    pub fn on_time(&mut self, h: u8, mi: u8, s: u8) -> Option<SyncEpoch> {
        let (y, mo, d) = self.last_date?;
        let (capture_ns, query_ns) = self.pending.take()?;
        // UTC second of the PPS edge = NMEA second + receiver-dependent correction (absorbs ±1 s).
        let unix_s = civil_to_unix(y as i64, mo as i64, d as i64, h as i64, mi as i64, s as i64)
            + self.association.edge_offset_seconds();
        Some(SyncEpoch {
            capture_ns,
            query_ns,
            unix_ns: unix_s * 1_000_000_000,
        })
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
    fn parse_zda_extracts_time_and_a_four_digit_year() {
        // A real sentence from the receiver this was developed against.
        let s = "$GPZDA,024247.000,18,08,2026,,*56";
        let r = parse_zda_time_date(s).unwrap();
        assert_eq!(r.0, (2, 42, 47));
        // No century to guess at, unlike RMC's two-digit year.
        assert_eq!(r.1, (18, 8, 2026));
    }

    #[test]
    fn parse_zda_ignores_other_sentences() {
        // Both appear in the same burst, so a parser that accepted either would make the choice of
        // time source meaningless — whichever arrived first would win.
        let rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
        assert!(parse_zda_time_date(rmc).is_none());
        assert!(
            parse_zda_time_date("$GPGGA,024247.000,3605.7502,N,1,11,1.11,30.9,M,,,*5A").is_none()
        );
    }

    #[test]
    fn parse_zda_rejects_impossible_dates() {
        assert!(parse_zda_time_date("$GPZDA,024247.000,32,08,2026,,*00").is_none());
        assert!(parse_zda_time_date("$GPZDA,024247.000,18,13,2026,,*00").is_none());
        assert!(parse_zda_time_date("$GPZDA,024247.000,00,08,2026,,*00").is_none());
        // A year before GPS existed means the field is not what we think it is.
        assert!(parse_zda_time_date("$GPZDA,024247.000,18,08,1970,,*00").is_none());
        // ZDA's year is four digits; other widths mean the fields are not where we think.
        assert!(parse_zda_time_date("$GPZDA,024247.000,18,08,26,,*00").is_none());
        assert!(parse_zda_time_date("$GPZDA,024247.000,18,08,20260,,*00").is_none());
    }

    #[test]
    fn parse_zda_rejects_a_day_past_the_end_of_its_month() {
        // `day <= 31` alone would let these through, and `civil_to_unix` normalises them into the
        // following month rather than failing — an epoch silently days off.
        assert!(parse_zda_time_date("$GPZDA,024247.000,31,02,2026,,*00").is_none());
        assert!(parse_zda_time_date("$GPZDA,024247.000,31,04,2026,,*00").is_none());
        assert!(parse_zda_time_date("$GPZDA,024247.000,31,09,2026,,*00").is_none());
        // The last valid day of a 30-day month still parses.
        assert!(parse_zda_time_date("$GPZDA,024247.000,30,04,2026,,*00").is_some());
    }

    #[test]
    fn parse_zda_knows_which_februaries_have_29_days() {
        // 2024 is a leap year, 2026 is not, 2000 is (divisible by 400), 1900 would not be.
        assert!(parse_zda_time_date("$GPZDA,024247.000,29,02,2024,,*00").is_some());
        assert!(parse_zda_time_date("$GPZDA,024247.000,29,02,2026,,*00").is_none());
        assert!(parse_zda_time_date("$GPZDA,024247.000,29,02,2000,,*00").is_some());
        assert!(parse_zda_time_date("$GPZDA,024247.000,29,02,2100,,*00").is_none());
    }

    #[test]
    fn parse_zda_rejects_a_truncated_sentence() {
        assert!(parse_zda_time_date("$GPZDA,024247.000,18").is_none());
        assert!(parse_zda_time_date("$GPZDA").is_none());
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

    // 1 s on the capture timebase, in ns (the tests use whole-second edges for clarity).
    const NS: i64 = 1_000_000_000;

    #[test]
    fn epoch_requires_both_date_and_edge() {
        let mut ts = PpsTimeSync::new();
        assert!(ts.on_time(17, 6, 58).is_none()); // nothing yet
        ts.on_pps_edge(1_000_000_000, 5_000_000_000);
        assert!(ts.on_time(17, 6, 58).is_none()); // date not yet set
        ts.set_date(2026, 6, 7);
        let e = ts.on_time(17, 6, 58).unwrap();
        assert_eq!(e.capture_ns, 1_000_000_000);
        assert_eq!(e.query_ns, 5_000_000_000); // the two timebases are kept independent
        assert_eq!(e.unix_ns, civil_to_unix(2026, 6, 7, 17, 6, 58) * NS);
    }

    #[test]
    fn edge_paired_once_then_consumed() {
        let mut ts = PpsTimeSync::new();
        ts.set_date(2026, 6, 7);
        ts.on_pps_edge(1_000_000_000, 1_000_000_000);
        assert!(ts.on_time(17, 6, 58).is_some());
        // No new edge: the stale edge is not paired to a second second (would be a ±1 s epoch jump).
        assert!(ts.on_time(17, 6, 59).is_none());
        // A fresh edge pairs again.
        ts.on_pps_edge(2_000_000_000, 2_000_000_000);
        assert!(ts.on_time(17, 6, 59).is_some());
    }

    #[test]
    fn association_next_second_shifts_epoch_back() {
        // The receiver sends the "next" second early: the edge's UTC second is NMEA second − 1.
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsNextSecond);
        ts.set_date(2026, 6, 7);
        ts.on_pps_edge(1_000_000_000, 1_000_000_000);
        let e = ts.on_time(17, 6, 58).unwrap();
        // The default (SameSecond) would be 17:06:58, but NmeaIsNextSecond maps it to 17:06:57.
        assert_eq!(e.unix_ns, civil_to_unix(2026, 6, 7, 17, 6, 57) * NS);
    }

    #[test]
    fn association_previous_second_shifts_epoch_forward() {
        // The receiver reports the "previous" second: the edge's UTC second is NMEA second + 1.
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsPreviousSecond);
        ts.set_date(2026, 6, 7);
        ts.on_pps_edge(1_000_000_000, 1_000_000_000);
        let e = ts.on_time(17, 6, 58).unwrap();
        assert_eq!(e.unix_ns, civil_to_unix(2026, 6, 7, 17, 6, 59) * NS);
    }

    #[test]
    fn association_handles_day_rollover() {
        // 00:00:00 with NmeaIsNextSecond(-1) → previous day 23:59:59 (Unix-second math rolls over).
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsNextSecond);
        ts.set_date(2026, 6, 7);
        ts.on_pps_edge(5_000_000_000, 5_000_000_000);
        let e = ts.on_time(0, 0, 0).unwrap();
        assert_eq!(e.unix_ns, civil_to_unix(2026, 6, 6, 23, 59, 59) * NS);
    }

    #[test]
    fn association_offset_does_not_accumulate() {
        // Across consecutive syncs the correction does not accumulate (not +1 per second; it applies
        // equally to every epoch).
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsNextSecond);
        ts.set_date(2026, 6, 7);
        ts.on_pps_edge(1_000_000_000, 1_000_000_000);
        let e1 = ts.on_time(17, 6, 58).unwrap();
        ts.on_pps_edge(2_000_000_000, 2_000_000_000);
        let e2 = ts.on_time(17, 6, 59).unwrap();
        assert_eq!(e2.unix_ns - e1.unix_ns, NS);
        assert_eq!(e1.unix_ns, civil_to_unix(2026, 6, 7, 17, 6, 57) * NS);
        assert_eq!(e2.unix_ns, civil_to_unix(2026, 6, 7, 17, 6, 58) * NS);
    }

    #[test]
    fn epoch_advances_each_second() {
        let mut ts = PpsTimeSync::new();
        ts.set_date(2026, 6, 7);
        ts.on_pps_edge(1_000_000_000, 1_000_000_000);
        let e1 = ts.on_time(17, 6, 58).unwrap();
        ts.on_pps_edge(2_000_000_000, 2_000_000_000);
        let e2 = ts.on_time(17, 6, 59).unwrap();
        assert_eq!(e2.unix_ns - e1.unix_ns, NS);
        assert_eq!(e2.capture_ns, 2_000_000_000);
    }
}
