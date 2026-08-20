//! An all-in-one GPSDO state bundle: PPS edge + NMEA time → disciplined UTC (`gnssdo` feature).
//!
//! [`PpsGpsdo`] bundles the discipline core ([`gnssdo::Gnssdo`]) with the PPS↔NMEA epoch pairing
//! ([`PpsTimeSync`](crate::PpsTimeSync)) behind one object, so a caller feeds it timed PPS edges and
//! framed NMEA sentences and reads disciplined UTC — without wiring `Gnssdo`, `PpsTimeSync` and the
//! residual diagnostics together by hand.
//!
//! It deliberately does **not** own the capture/output I/O or any executor: the caller still owns
//! the [`TimedPpsCapture`](crate::embassy::TimedPpsCapture) (so it can use the raw counter for, e.g.,
//! a loopback phase measurement) and the timebases, and drives this from its own tasks — typically
//! `on_pps_edge` from a PPS task and [`feed_nmea`](PpsGpsdo::feed_nmea) from the UART task, behind a
//! mutex. Logging, receiver config and pin/SM assignment stay the caller's.

use crate::{TimedEdge, parse_rmc_time_date, parse_zda_time_date};
use gnssdo::{Gnssdo, GnssdoStep};

/// Which NMEA sentence supplies the UTC second that gets paired with a PPS edge.
///
/// This is not a matter of taste. **ZDA is the only sentence specified against the timing pulse**:
/// the MT3333 platform NMEA specification lists it as the "PPS timing message (synchronized to
/// PPS)" and states that it "outputs the time associated with the current 1PPS pulse … and tells
/// the time of the pulse that just occurred". RMC also carries a time, but no specification ties it
/// to the pulse — it is a navigation sentence that happens to contain a clock, and it sits near the
/// end of the NMEA burst where, at 9600 baud, it can arrive *after the next edge*.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum NmeaTimeSource {
    /// **R**ecommended **M**inimum Navigation Information — and the name is the argument.
    ///
    /// Still expanded that way in NMEA 0183 v4.11: the minimum set of data a *navigation* source
    /// should provide — position, course, speed, and the time those apply to. It sits beside RMA
    /// (Loran-C) and RMB (navigation to a waypoint) from the same era. It carries a clock because
    /// navigation needs one, not because it was designed to state the time.
    ///
    /// Select this only for a receiver that does not emit ZDA. Its time is not defined against the
    /// timing pulse, so pairing it with an edge works by convention rather than by specification.
    Rmc,
    /// Time & Date — the sentence defined against the 1PPS output. **The default.**
    #[default]
    Zda,
}

/// Diagnostics from the sync that [`PpsGpsdo::feed_nmea`] establishes (computed on the pre-update
/// clock, then the epoch is applied). All ns; the caller decides what/whether to log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyncReport {
    /// Capture-timebase value (ns) of the PPS edge this sync pinned.
    pub capture_ns: u64,
    /// The UTC instant (Unix ns) the edge was pinned to.
    pub unix_ns: i64,
    /// Prediction residual: post-correction time error at the edge (self-diagnostic, ns).
    pub err_ns: i64,
    /// `fire_at_utc` inverse-prediction residual (ns).
    pub fire_ns: i64,
    /// How long since the previous established epoch (= holdover this sync spans, ns).
    pub holdover_ns: u64,
    /// Crystal frequency estimate after this update (ppb).
    pub freq_ppb: i64,
}

/// PPS edge + NMEA time → disciplined UTC: the all-in-one easy tier bundling [`gnssdo::Gnssdo`]
/// (frequency discipline + holdover) with the PPS↔NMEA epoch pairing
/// ([`PpsTimeSync`](crate::PpsTimeSync)) and the residual diagnostics, behind one object.
///
/// It does **not** own the capture/output I/O or any executor: the caller keeps the capture (e.g.
/// `TimedPpsCapture`, so the raw counter stays available for a loopback phase measurement) and the
/// timebases, and drives this from its own tasks — typically [`on_pps_edge`](Self::on_pps_edge) from
/// a PPS task and [`feed_nmea`](Self::feed_nmea) from the UART task, behind a mutex. Logging,
/// receiver config and pin/SM assignment stay the caller's. See the `gpsdo` (drive by hand) and
/// `gpsdo_runner` (runner tasks) examples.
#[derive(Debug, Default)]
pub struct PpsGpsdo {
    clock: Gnssdo,
    sync: crate::PpsTimeSync,
    source: NmeaTimeSource,
}

impl PpsGpsdo {
    /// Create with the default discipline, [`ZDA`](NmeaTimeSource::Zda) as the time source, and the
    /// [`SameSecond`](crate::PpsNmeaAssociation::SameSecond) association.
    ///
    /// Those two defaults belong together: ZDA is specified to report "the time of the pulse that
    /// just occurred", which *is* `SameSecond`. Use [`with_config`](Self::with_config) for a
    /// receiver that does not emit ZDA.
    pub const fn new() -> Self {
        Self::with_association(crate::PpsNmeaAssociation::SameSecond)
    }

    /// Create with an explicit time source and PPS↔NMEA association.
    ///
    /// See [`NmeaTimeSource`] — on a receiver that emits ZDA, preferring it is the more defensible
    /// choice, because ZDA is the only sentence specified against the timing pulse.
    pub const fn with_config(
        source: NmeaTimeSource,
        association: crate::PpsNmeaAssociation,
    ) -> Self {
        Self {
            clock: Gnssdo::new(),
            sync: crate::PpsTimeSync::with_association(association),
            source,
        }
    }

    /// Create with an explicit PPS↔NMEA association.
    ///
    /// Worth reaching for: [`PpsNmeaAssociation`](crate::PpsNmeaAssociation) is documented as the
    /// biggest footgun in this library, because getting it wrong shifts the whole UTC epoch by a
    /// second while every *phase* measurement still looks perfect. Until this existed, the turn-key
    /// bundle was the one path that could not change it.
    ///
    /// The symptom is invisible without an external time reference — a disciplined 1PPS output can
    /// sit on the GPS edge to nanoseconds and still be labelled with the wrong second.
    pub const fn with_association(association: crate::PpsNmeaAssociation) -> Self {
        Self::with_config(NmeaTimeSource::Zda, association)
    }

    /// Feed a timed PPS edge, timestamped on the query timebase (e.g. embassy `Instant`) as
    /// `query_ns`. Disciplines the crystal frequency and records the edge for the next
    /// [`feed_nmea`](Self::feed_nmea) pairing. Returns the [`GnssdoStep`] for logging.
    pub fn on_pps_edge(&mut self, edge: TimedEdge, query_ns: u64) -> GnssdoStep {
        self.sync.on_pps_edge(edge.edge_ns, query_ns);
        self.clock
            .on_pps(edge.edge_ns / 1000, edge.interval_ns as i64)
    }

    /// Feed a framed NMEA sentence.
    ///
    /// Only the sentence named by the configured [`NmeaTimeSource`] is considered — RMC by default,
    /// ZDA with [`with_config`](Self::with_config). When that sentence arrives and a fresh PPS edge
    /// is pending, the UTC epoch is established or refreshed and a [`SyncReport`] is returned.
    ///
    /// `None` otherwise, which covers three different situations that a caller may want to tell
    /// apart by other means: the sentence is not the configured one (including the *other* time
    /// sentence, which is ignored rather than used), no fresh edge is pending, or the sentence
    /// failed to parse. In none of these is the pending edge consumed, so a later matching sentence
    /// can still pair with it.
    pub fn feed_nmea(&mut self, sentence: &str) -> Option<SyncReport> {
        // Only the configured sentence is parsed, never "whichever arrives first". Both RMC and ZDA
        // appear in the same burst, and `on_time` consumes the pending edge — so accepting both
        // would silently let the earlier one win and make the choice meaningless.
        let ((h, mi, s), (day, month, year)) = match self.source {
            NmeaTimeSource::Rmc => parse_rmc_time_date(sentence)?,
            NmeaTimeSource::Zda => parse_zda_time_date(sentence)?,
        };
        self.sync.set_date(year, month, day);
        let epoch = self.sync.on_time(h, mi, s)?;
        // Residuals are computed on the pre-update clock (the self-diagnostic of the correction).
        let err_ns = self
            .clock
            .clock()
            .prediction_residual_ns(epoch.capture_ns, epoch.unix_ns)
            .unwrap_or(0);
        let fire_ns = self
            .clock
            .clock()
            .fire_residual_ns(epoch.capture_ns, epoch.unix_ns)
            .unwrap_or(0);
        let holdover_ns = self.clock.holdover_ns(epoch.query_ns);
        self.clock
            .on_utc(epoch.capture_ns, epoch.query_ns, epoch.unix_ns);
        Some(SyncReport {
            capture_ns: epoch.capture_ns,
            unix_ns: epoch.unix_ns,
            err_ns,
            fire_ns,
            holdover_ns,
            freq_ppb: self.clock.freq_ppb(),
        })
    }

    /// Disciplined UTC (Unix ns) for a query-timebase value; holdover-extrapolated. `None` until an
    /// epoch is established.
    pub fn now_from_query_ns(&self, query_ns: u64) -> Option<i64> {
        self.clock.now_from_query_ns(query_ns)
    }

    /// UTC for a moment on the **capture counter**, rather than on the software clock.
    ///
    /// The 1PPS output knows where its edges fall on that counter and nowhere else: its state
    /// machine was started by the same write, and every edge since is a period word this side
    /// counted out. Asking in the query timebase instead means converting through a software clock
    /// read, and whatever that read cost lands on the output as a fixed offset.
    ///
    /// `None` until an epoch is established, like [`now_from_query_ns`](Self::now_from_query_ns).
    pub fn now_from_capture_ns(&self, capture_ns: u64) -> Option<i64> {
        self.clock.now_from_capture_ns(capture_ns)
    }

    /// Estimated crystal frequency offset (ppb).
    pub fn freq_ppb(&self) -> i64 {
        self.clock.freq_ppb()
    }

    /// Feed a temperature reading (raw sensor units) for the temperature feedforward. Passthrough to
    /// [`Gnssdo::update_temp`]; call once per edge alongside the frequency discipline.
    pub fn update_temp(&mut self, temp: i64) {
        self.clock.update_temp(temp);
    }

    /// Toggle the temperature feedforward at runtime. Passthrough to [`Gnssdo::set_temp_ff_enable`].
    pub fn set_temp_ff_enable(&mut self, en: bool) {
        self.clock.set_temp_ff_enable(en);
    }

    /// Tune the temperature feedforward at runtime. Passthrough to [`Gnssdo::set_temp_ff_params`].
    pub fn set_temp_ff_params(&mut self, sm_shift: u32, shift: u32, gain_q8: i64) {
        self.clock.set_temp_ff_params(sm_shift, shift, gain_q8);
    }

    /// Tune the matched-lead + residual-observer knobs at runtime. Passthrough to
    /// [`Gnssdo::set_temp_ff_lag`].
    pub fn set_temp_ff_lag(&mut self, lag_q8: i64, dlead_shift: u32, obs_shift: u32) {
        self.clock.set_temp_ff_lag(lag_q8, dlead_shift, obs_shift);
    }

    /// Crystal frequency (milli-ppb) projected one sample ahead — the value to feed the **output
    /// period**. Full mppb resolution (no ppb rounding) and slope-projected, so a temperature-driven
    /// frequency ramp does not leave a standing output-phase error (see
    /// [`DisciplinedClock::predicted_freq_mppb`](gnssdo::DisciplinedClock::predicted_freq_mppb)).
    pub fn predicted_freq_mppb(&self) -> i64 {
        self.clock.clock().predicted_freq_mppb()
    }

    /// Crystal frequency (milli-ppb) to feed the **output period steering**: like
    /// [`predicted_freq_mppb`](Self::predicted_freq_mppb) but with the feedforward deviation from the
    /// α-β level bounded to ±[`steer_ff_bound_mppb`](gnssdo::DisciplinedClockConfig::steer_ff_bound_mppb)
    /// so a fast thermal transient where the matched-lead `predicted` over-reacts cannot slam the
    /// output period. `predicted_freq_mppb` is left raw for holdover; only steering is clamped. Steady
    /// operation (deviation ≤ 5 ppb) is bit-identical to `predicted_freq_mppb`. Passthrough to
    /// [`DisciplinedClock::steering_freq_mppb`](gnssdo::DisciplinedClock::steering_freq_mppb).
    pub fn steering_freq_mppb(&self) -> i64 {
        self.clock.clock().steering_freq_mppb()
    }

    /// Tracked frequency slope (milli-ppb per sample): how fast the crystal offset is drifting (a
    /// temperature-ramp proxy). 0 in steady state. For logging.
    pub fn freq_slope_mppb(&self) -> i64 {
        self.clock.clock().freq_slope_mppb()
    }

    /// α-β frequency **level** (milli-ppb), *without* the temperature-feedforward lead. The temp-FF
    /// steering contribution is `predicted_freq_mppb() − freq_mppb()`; logging both lets an experiment
    /// confirm the feedforward is actually active and measure its magnitude.
    pub fn freq_mppb(&self) -> i64 {
        self.clock.clock().freq_mppb()
    }

    /// Learned temperature coefficient `k` (milli-ppb per raw temperature unit), 0 until temperature
    /// has varied enough for the online regression. For logging/diagnostics of the temperature
    /// feedforward.
    pub fn temp_k_mppb_per_unit(&self) -> i64 {
        self.clock.clock().temp_k_mppb_per_unit()
    }

    /// Nanoseconds since the last established epoch (holdover span at `query_ns`).
    pub fn holdover_ns(&self, query_ns: u64) -> u64 {
        self.clock.holdover_ns(query_ns)
    }

    /// Whether the frequency estimate has locked.
    pub fn frequency_locked(&self) -> bool {
        self.clock.frequency_locked()
    }

    /// Borrow the bundled [`Gnssdo`] (the fine tier — e.g. for `clock()` residual APIs or config).
    pub fn gnssdo(&self) -> &Gnssdo {
        &self.clock
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // A whole-second RMC at 2026-06-07 17:06:58 (date 070626; yy=26 → 2026 on both parsers).
    // Valid `*6F` checksum so the test also passes under the `external-nmea` (validating) parser.
    const RMC: &str = "$GPRMC,170658.000,A,3541.0,N,13945.0,E,0.0,0.0,070626,,,A*6F";

    /// A clock configured for RMC, for the tests that are about RMC specifically.
    fn rmc_gpsdo() -> PpsGpsdo {
        PpsGpsdo::with_config(NmeaTimeSource::Rmc, crate::PpsNmeaAssociation::SameSecond)
    }

    fn edge(ns: u64) -> TimedEdge {
        TimedEdge {
            raw: 0,
            interval_ns: 1_000_000_000,
            edge_ns: ns,
        }
    }

    #[test]
    fn feed_nmea_needs_a_fresh_edge() {
        let mut g = rmc_gpsdo();
        // No edge yet → no epoch, no disciplined time.
        assert!(g.feed_nmea(RMC).is_none());
        assert!(g.now_from_query_ns(1_000_000_000).is_none());
    }

    #[test]
    fn utc_can_be_asked_for_on_the_capture_counter() {
        // The counter and the software clock do not share an origin, and only the counter is what
        // the output schedule counts in. Here the edge is at 1.0 s on the counter and 1.5 s on the
        // software clock, so asking the wrong one is off by half a second.
        let mut g = rmc_gpsdo();
        g.on_pps_edge(edge(1_000_000_000), 1_500_000_000);
        let want = crate::civil_to_unix(2026, 6, 7, 17, 6, 58) * 1_000_000_000;
        assert_eq!(g.feed_nmea(RMC).map(|r| r.unix_ns), Some(want));
        assert_eq!(
            g.now_from_capture_ns(1_000_000_000),
            Some(want),
            "the edge itself, on the counter"
        );
        assert_eq!(
            g.now_from_capture_ns(1_500_000_000),
            Some(want + 500_000_000),
            "half a second of counter is half a second of UTC"
        );
    }

    #[test]
    fn edge_then_rmc_establishes_utc() {
        let mut g = rmc_gpsdo();
        g.on_pps_edge(edge(1_000_000_000), 1_000_000_000);
        let report = g
            .feed_nmea(RMC)
            .expect("RMC + fresh edge establishes a sync");
        let want = crate::civil_to_unix(2026, 6, 7, 17, 6, 58) * 1_000_000_000;
        assert_eq!(report.unix_ns, want);
        assert_eq!(report.capture_ns, 1_000_000_000);
        // The epoch maps the edge's query time to the RMC second.
        assert_eq!(g.now_from_query_ns(1_000_000_000), Some(want));
        // A stale edge is not re-paired (no new on_pps_edge): no second sync.
        assert!(g.feed_nmea(RMC).is_none());
    }

    // --- The configured time source, end to end ---
    //
    // The parser tests next door only prove `parse_zda_time_date` reads a sentence. These prove the
    // thing this configuration exists for: that selecting ZDA actually changes which sentence
    // establishes the epoch, and that the *other* time sentence is passed over rather than used.

    /// The same instant as `RMC` above, as ZDA. Checksum is genuine, though the built-in parser
    /// does not validate it.
    const ZDA: &str = "$GPZDA,170658.000,07,06,2026,,*5C";

    #[test]
    fn zda_source_establishes_utc_from_zda() {
        let mut g =
            PpsGpsdo::with_config(NmeaTimeSource::Zda, crate::PpsNmeaAssociation::SameSecond);
        g.on_pps_edge(edge(1_000_000_000), 1_000_000_000);
        let report = g
            .feed_nmea(ZDA)
            .expect("ZDA + fresh edge establishes a sync");
        let want = crate::civil_to_unix(2026, 6, 7, 17, 6, 58) * 1_000_000_000;
        assert_eq!(report.unix_ns, want);
        assert_eq!(g.now_from_query_ns(1_000_000_000), Some(want));
    }

    #[test]
    fn zda_source_passes_over_rmc_without_consuming_the_edge() {
        // RMC arrives *earlier* in the burst than ZDA. If it were accepted — or if rejecting it
        // still consumed the pending edge — the ZDA that follows would find nothing to pair with
        // and the whole selection would be decorative.
        let mut g =
            PpsGpsdo::with_config(NmeaTimeSource::Zda, crate::PpsNmeaAssociation::SameSecond);
        g.on_pps_edge(edge(1_000_000_000), 1_000_000_000);
        assert!(
            g.feed_nmea(RMC).is_none(),
            "RMC must not establish an epoch"
        );
        assert!(
            g.now_from_query_ns(1_000_000_000).is_none(),
            "and must not have established one behind our back"
        );
        // The edge survived, so the sentence we actually asked for can still use it.
        let report = g
            .feed_nmea(ZDA)
            .expect("the edge was still pending for ZDA");
        assert_eq!(
            report.unix_ns,
            crate::civil_to_unix(2026, 6, 7, 17, 6, 58) * 1_000_000_000
        );
    }

    #[test]
    fn the_default_time_source_is_zda() {
        // The point of the change that made it so: a caller who does not think about this at all
        // should get the sentence that is defined against the pulse, not the one that merely
        // happens to carry a clock.
        let mut g = PpsGpsdo::new();
        g.on_pps_edge(edge(1_000_000_000), 1_000_000_000);
        assert!(
            g.feed_nmea(RMC).is_none(),
            "RMC must not establish an epoch by default"
        );
        assert!(g.feed_nmea(ZDA).is_some(), "ZDA must");
    }

    #[test]
    fn rmc_source_passes_over_zda_symmetrically() {
        let mut g = rmc_gpsdo();
        g.on_pps_edge(edge(1_000_000_000), 1_000_000_000);
        assert!(
            g.feed_nmea(ZDA).is_none(),
            "ZDA must not establish an epoch"
        );
        assert!(g.feed_nmea(RMC).is_some(), "the edge was still pending");
    }

    #[test]
    fn the_configured_association_shifts_the_epoch() {
        // The ±1 s correction is the setting this crate calls its biggest footgun, and until
        // `with_config`/`with_association` existed the bundle could not reach it at all. Prove it
        // arrives: the same sentence and edge must land a second later.
        let mut g = PpsGpsdo::with_config(
            NmeaTimeSource::Zda,
            crate::PpsNmeaAssociation::NmeaIsPreviousSecond,
        );
        g.on_pps_edge(edge(1_000_000_000), 1_000_000_000);
        let report = g
            .feed_nmea(ZDA)
            .expect("ZDA + fresh edge establishes a sync");
        let same_second = crate::civil_to_unix(2026, 6, 7, 17, 6, 58) * 1_000_000_000;
        assert_eq!(
            report.unix_ns,
            same_second + 1_000_000_000,
            "NmeaIsPreviousSecond puts the edge one second after the sentence"
        );
    }

    #[test]
    fn on_pps_edge_ignores_raw_counter() {
        // The software (non-PIO) path feeds TimedEdge { raw: 0, .. }: on_pps_edge must depend only on
        // edge_ns / interval_ns, never the raw down-counter. Drive two clocks with identical timing
        // but different (and zero) raw values and require identical discipline.
        let mut a = PpsGpsdo::new();
        let mut b = PpsGpsdo::new();
        for i in 0..10u64 {
            let edge_ns = (i + 1) * 1_000_000_000;
            let raw_b = 0x1234_5678u32.wrapping_add((i as u32).wrapping_mul(0x9E37_79B9));
            a.on_pps_edge(
                TimedEdge {
                    raw: 0,
                    interval_ns: 1_000_000_000,
                    edge_ns,
                },
                edge_ns,
            );
            b.on_pps_edge(
                TimedEdge {
                    raw: raw_b,
                    interval_ns: 1_000_000_000,
                    edge_ns,
                },
                edge_ns,
            );
        }
        assert_eq!(a.freq_ppb(), b.freq_ppb());
        assert_eq!(a.predicted_freq_mppb(), b.predicted_freq_mppb());
        assert_eq!(
            a.now_from_query_ns(10_000_000_000),
            b.now_from_query_ns(10_000_000_000)
        );
    }

    #[test]
    fn non_rmc_is_ignored() {
        let mut g = PpsGpsdo::new();
        g.on_pps_edge(edge(1_000_000_000), 1_000_000_000);
        assert!(g.feed_nmea("$GPGGA,170658.000,3541.0,N*00").is_none());
    }
}
