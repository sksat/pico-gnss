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

use crate::{TimedEdge, parse_rmc_time_date};
use gnssdo::{Gnssdo, GnssdoStep};

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
}

impl PpsGpsdo {
    /// Create with the default discipline and [`SameSecond`](crate::PpsNmeaAssociation::SameSecond)
    /// PPS↔NMEA association.
    pub const fn new() -> Self {
        Self {
            clock: Gnssdo::new(),
            sync: crate::PpsTimeSync::new(),
        }
    }

    /// Feed a timed PPS edge, timestamped on the query timebase (e.g. embassy `Instant`) as
    /// `query_ns`. Disciplines the crystal frequency and records the edge for the next
    /// [`feed_nmea`](Self::feed_nmea) pairing. Returns the [`GnssdoStep`] for logging.
    pub fn on_pps_edge(&mut self, edge: TimedEdge, query_ns: u64) -> GnssdoStep {
        self.sync.on_pps_edge(edge.edge_ns, query_ns);
        self.clock
            .on_pps(edge.edge_ns / 1000, edge.interval_ns as i64)
    }

    /// Feed a framed NMEA sentence. On an RMC paired with a fresh PPS edge, establishes/refreshes
    /// the UTC epoch and returns a [`SyncReport`]; otherwise (non-RMC, no fresh edge, parse failure)
    /// returns `None`.
    pub fn feed_nmea(&mut self, sentence: &str) -> Option<SyncReport> {
        let ((h, mi, s), (day, month, year)) = parse_rmc_time_date(sentence)?;
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

    fn edge(ns: u64) -> TimedEdge {
        TimedEdge {
            raw: 0,
            interval_ns: 1_000_000_000,
            edge_ns: ns,
        }
    }

    #[test]
    fn feed_nmea_needs_a_fresh_edge() {
        let mut g = PpsGpsdo::new();
        // No edge yet → no epoch, no disciplined time.
        assert!(g.feed_nmea(RMC).is_none());
        assert!(g.now_from_query_ns(1_000_000_000).is_none());
    }

    #[test]
    fn edge_then_rmc_establishes_utc() {
        let mut g = PpsGpsdo::new();
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

    #[test]
    fn non_rmc_is_ignored() {
        let mut g = PpsGpsdo::new();
        g.on_pps_edge(edge(1_000_000_000), 1_000_000_000);
        assert!(g.feed_nmea("$GPGGA,170658.000,3541.0,N*00").is_none());
    }
}
