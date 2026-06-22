//! Clock discipline for a GPS-disciplined oscillator (GPSDO).
//!
//! EMA-estimates the crystal's frequency offset from the PPS interval (ns precision from a
//! hardware capture, e.g. RP2040 PIO) and, together with a UTC epoch, provides "disciplined UTC"
//! on the device. The essence of a GPSDO is that **even while PPS is lost (holdover), it keeps
//! time by extrapolating with the estimated frequency**.
//!
//! Units are ppb (parts per billion = ns/s). The PPS interval deviation (interval_ns − 1e9) is
//! directly the frequency offset in ppb (1 ppb = 1 ns/s).
//!
//! The frequency estimate comes from the precise PPS interval, while the epoch (the absolute-time
//! anchor) comes from a continuously-readable local clock (e.g. embassy Instant); they are given
//! separately and only need to share the unit (ns). All of this is HAL-agnostic, so it is
//! host-tested (`cargo test -p gnssdo`).

use core::num::{NonZeroU32, NonZeroU64};

use crate::pps::{PpsEvent, PpsTracker, PpsTrackerConfig};

/// Tuning configuration for [`DisciplinedClock`].
///
/// The right values depend on the receiver, antenna, TCXO/crystal, PPS capture resolution, and
/// operating mode (stationary/moving), so they are configurable rather than fixed consts. `Default`
/// (= [`DisciplinedClockConfig::DEFAULT`]) are the measured settling values for a GYSFFMANC (MT3333)
/// at a fixed window position with PIO ns capture.
///
/// **Invalid values are excluded by the type**: fields that are meaningless/broken at 0 or negative
/// (sample counts, the gates) are `NonZero*`, so an invalid value cannot even be constructed. Only
/// the shift amounts have an upper-bound constraint (0..=63), which `NonZero` cannot express, so
/// they are clamped with `min(63)` at use (to avoid i64 shift overflow).
///
/// (`Debug`/`Default` is the minimum required by [`DisciplinedClock`]'s derive; `Clone`/`PartialEq`
///  etc. are not added until comparison or cloning is actually needed.)
#[derive(Debug)]
pub struct DisciplinedClockConfig {
    /// Post-lock EMA smoothing coefficient alpha = 1/2^`ema_shift` (default 5 ≈ 32-sample time
    /// constant). Valid range 0..=63; values above are clamped to 63 internally.
    pub ema_shift: u32,
    /// Faster EMA coefficient while converging (unlocked), alpha = 1/2^`ema_shift_fast` (default 3 =
    /// 1/8). Right after start-up there is no estimate yet so it captures quickly; after lock it
    /// smooths slowly to suppress jitter. Valid range 0..=63 (clamped like `ema_shift`).
    pub ema_shift_fast: u32,
    /// Number of frequency samples required to declare lock (default 8).
    pub lock_samples: NonZeroU32,
    /// **Emergency-stop** window (1s ± `sane_dev_ns`, default 1ms). Intervals outside this (the PIO
    /// ~68s wrap glitch's false short interval ≈ -37ms, or a ~2s gap) are clearly invalid. At 50ms
    /// the wrap glitch slipped through and contaminated the EMA (found in on-hardware evaluation).
    /// This is the last safety net rather than the normal quality check; normal quality is filtered
    /// by the converge/residual gates below (per smart-friend GPT-5.5).
    pub sane_dev_ns: NonZeroU64,
    /// Absolute quality gate while unlocked/converging (default ±100µs). The EMA is not yet
    /// trustworthy, so reject by absolute value. Prevents medium multipath at a weak window position
    /// (tens to hundreds of µs) from contaminating the EMA in the first few samples.
    pub converge_gate_ns: NonZeroU64,
    /// Post-lock residual gate: a single sample whose `|measured − EMA|` exceeds `residual_gate_ns`
    /// (default ±5µs) is treated as multipath and rejected. The true jitter at a fixed window
    /// position is ns to tens of ns, so 5µs is a generously safe margin.
    pub residual_gate_ns: NonZeroU64,
    /// Number of samples for which frequency-EMA updates are held off right after recovering from
    /// holdover/Irregular (default 5). The PPS right after recovery is untrustworthy (receiver
    /// internal state, NMEA association, PPS phase not yet settled). 0 = no quarantine (a valid
    /// setting, so it is not `NonZero`).
    pub quarantine_samples: u32,
    /// Slope-EMA shift for the α-β frequency tracker (default 9, much slower than `ema_shift`). The
    /// level (`freq_mppb`) is a single EMA; on its own it lags a frequency *ramp* (temperature drift)
    /// by ≈`2^ema_shift` samples, so the disciplined output runs at a stale frequency and accrues a
    /// phase error proportional to the ramp rate (the measured µs-scale offset② temperature drift).
    /// Estimating the slope `freq_slope_mppb` (per sample) and projecting it forward
    /// ([`predicted_freq_mppb`](DisciplinedClock::predicted_freq_mppb)) cancels that lag to first
    /// order. The slope must adapt far more slowly than the level or it becomes a jitter
    /// differentiator, hence the larger shift. Frozen while converging/quarantined. 0..=63 (clamped).
    pub slope_ema_shift: u32,
    /// Clamp on the tracked frequency slope (milli-ppb per sample, default 5 000 = 5 ppb/s). This is
    /// the **dominant safety bound on the feedforward**: a *sustained* crystal thermal drift is at
    /// most a few ppb/s (≈0.5 ppm/°C × a slow °C/s), so 5 ppb/s covers real warming with margin. A
    /// violent shock (a hair dryer ≈ tens of ppb/s) is faster than the once-per-second loop can track
    /// anyway, and feeding its full slope forward overshoots; clamping the slope here caps that
    /// feedforward (and bounds the stale slope left after the shock, which decays from the clamp).
    pub slope_max_mppb: i64,
    /// How many samples ahead [`predicted_freq_mppb`](DisciplinedClock::predicted_freq_mppb) projects
    /// the slope (default 1). The output period set this edge governs roughly the next 1 s, so a lead
    /// of one sample predicts the crystal's frequency over that interval. 0 = no feedforward (level
    /// only).
    pub pred_lead_samples: i64,
    /// Widen the post-lock residual gate by `|slope| * this` (default 8). During a fast ramp the
    /// honest measurement legitimately sits far from the level; without widening, the fixed
    /// `residual_gate_ns` would reject good samples and stall the tracker mid-ramp.
    pub residual_slope_widen: i64,
    /// Freeze slope adaptation when the prediction residual `|measured − projected|` exceeds this
    /// (milli-ppb, default 2 000 000 = 2 µs/s). A gentle thermal ramp leaves a small residual (the
    /// α-β tracks it), so the slope keeps adapting; a violent thermal shock leaves a large residual,
    /// where adapting the slope would spike it to tens of ppb/s and (because the slope is fed forward
    /// and decays slowly) leave a stale, wrong feedforward after the shock. Freezing the slope there
    /// keeps the feedforward sane through shocks. The *level* still updates; only the slope is held.
    pub slope_freeze_resid_mppb: i64,
}

impl DisciplinedClockConfig {
    /// The measured settling defaults. Held as `const` so the `const fn` [`DisciplinedClock::new`]
    /// can use it.
    pub const DEFAULT: Self = Self {
        ema_shift: 5,
        ema_shift_fast: 3,
        lock_samples: NonZeroU32::new(8).unwrap(),
        sane_dev_ns: NonZeroU64::new(1_000_000).unwrap(),
        converge_gate_ns: NonZeroU64::new(100_000).unwrap(),
        residual_gate_ns: NonZeroU64::new(5_000).unwrap(),
        quarantine_samples: 5,
        slope_ema_shift: 9,
        slope_max_mppb: 5_000,
        pred_lead_samples: 1,
        residual_slope_widen: 8,
        slope_freeze_resid_mppb: 2_000_000,
    };
}

impl Default for DisciplinedClockConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Result of `update_freq`. Used by the caller (e.g. pps_task) for logging/state management.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FreqUpdate {
    /// The EMA was updated.
    Applied,
    /// Rejected by the ±1ms emergency-stop (wrap glitch / gap).
    GatedSane,
    /// Rejected by a quality gate (absolute while converging / residual after lock).
    GatedQuality,
    /// EMA update held off because we are in the recovery quarantine.
    Quarantined,
}

impl FreqUpdate {
    /// Short tag for logging: `"ok"` / `"sane"` / `"gate"` / `"quar"`.
    pub fn as_str(&self) -> &'static str {
        match self {
            FreqUpdate::Applied => "ok",
            FreqUpdate::GatedSane => "sane",
            FreqUpdate::GatedQuality => "gate",
            FreqUpdate::Quarantined => "quar",
        }
    }
}

/// The PPS-disciplined clock model.
///
/// Works with two timebases (both from the device's same oscillator, passed as integer ns;
/// HAL-agnostic):
/// - **capture timebase**: high-resolution PPS edge capture. Used for err measurement and the ns
///   precision of `fire_at_utc`. On the RP2040 this is the PIO hardware capture (on other chips, a
///   timer input-capture, etc.).
/// - **query timebase**: a continuously-readable clock. Used for ticker/holdover. On the RP2040
///   this is embassy `Instant`.
///
/// The epochs of both are given together via [`update_epoch`](Self::update_epoch).
#[derive(Debug, Default)]
pub struct DisciplinedClock {
    config: DisciplinedClockConfig,
    freq_mppb: i64, // frequency-offset EMA level (milli-ppb = ppb*1000, kept for higher resolution)
    freq_slope_mppb: i64, // α-β slope: estimated change in freq_mppb per sample (milli-ppb/sample)
    samples: u32,
    quarantine: u32, // while >0 we are in the recovery quarantine (frequency-EMA updates held off)
    epoch_capture_ns: Option<u64>, // capture-timebase epoch (ns, for err/now_from_capture_ns)
    epoch_query_ns: Option<u64>, // query-timebase epoch (for continuous query/holdover)
    epoch_unix_ns: Option<i64>,
    last_query_ns: Option<u64>, // last disciplined query-timebase time (for holdover measurement)
}

/// Remove the integer-second offset and keep only the sub-second residual.
/// The corrected prediction residual (err) can be a huge ~Ns value on recovery after PPS was lost
/// for several seconds, because the PPS↔RMC pairing can be off by an integer second. Snapping to
/// the nearest integer second recovers the **true holdover residual** (sub-second) buried inside it
/// (e.g. 25_000_000_360 → 360ns = the error of a 25s holdover).
pub fn snap_to_second_ns(raw: i64) -> i64 {
    let secs = (raw + raw.signum() * 500_000_000) / 1_000_000_000;
    raw - secs * 1_000_000_000
}

impl DisciplinedClock {
    /// Create with the defaults ([`DisciplinedClockConfig::DEFAULT`]).
    pub const fn new() -> Self {
        Self::with_config(DisciplinedClockConfig::DEFAULT)
    }

    /// Create with a given configuration. `const fn` so it can be used in `static` init.
    pub const fn with_config(config: DisciplinedClockConfig) -> Self {
        Self {
            config,
            freq_mppb: 0,
            freq_slope_mppb: 0,
            samples: 0,
            quarantine: 0,
            epoch_capture_ns: None,
            epoch_query_ns: None,
            epoch_unix_ns: None,
            last_query_ns: None,
        }
    }

    /// Current configuration.
    pub fn config(&self) -> &DisciplinedClockConfig {
        &self.config
    }

    /// EMA-update the frequency offset from a precise PPS interval (ns, ideally 1e9).
    ///
    /// Multi-stage gating rejects weak-signal (window multipath), wrap glitches, and the suspect PPS
    /// right after recovery:
    /// 1. **Emergency stop**: outside ±1ms → reject (wrap/gap).
    /// 2. **Recovery quarantine**: in progress → hold off (the past estimate is more trustworthy).
    /// 3. **Quality gate**: absolute ±100µs while unlocked; EMA residual ±5µs after lock.
    /// 4. EMA update (fast alpha while converging, slow after lock).
    ///
    /// **Caller's responsibility**: call this only on edges that `PpsTracker` classifies as `Locked`
    /// (do not feed Irregular/First intervals into the frequency estimate). On recovery, call
    /// `start_quarantine()`.
    pub fn update_freq(&mut self, interval_ns: i64) -> FreqUpdate {
        let dev = interval_ns - 1_000_000_000; // = ppb
        // 1. Emergency stop: outside ±1ms is a wrap glitch / gap. Always reject.
        if dev.abs() >= self.config.sane_dev_ns.get() as i64 {
            return FreqUpdate::GatedSane;
        }
        // 2. Recovery quarantine: for the first M samples after recovery the receiver state / PPS
        //    phase are not aligned and untrustworthy, so don't update the EMA; keep the past
        //    crystal-frequency estimate.
        if self.quarantine > 0 {
            self.quarantine -= 1;
            return FreqUpdate::Quarantined;
        }
        let measured_mppb = dev * 1000;
        // The very first sample has no gate reference (EMA), so accept it if it's within the
        // emergency-stop window.
        if self.samples == 0 {
            self.freq_mppb = measured_mppb;
            self.samples = 1;
            return FreqUpdate::Applied;
        }
        // 3. Quality gate.
        if self.is_locked() {
            // After lock: a single sample with a large residual from the level is multipath. Reject
            // far more tightly than ±1ms. Widen by |slope| so a fast (but real) ramp, whose honest
            // measurement legitimately sits far from the level, is not rejected and stalled.
            let gate = self.config.residual_gate_ns.get() as i64 * 1000
                + self.freq_slope_mppb.abs() * self.config.residual_slope_widen;
            if (measured_mppb - self.freq_mppb).abs() > gate {
                return FreqUpdate::GatedQuality;
            }
        } else {
            // While converging: the EMA is not yet trustworthy, so reject medium multipath by an
            // absolute gate.
            if dev.abs() > self.config.converge_gate_ns.get() as i64 {
                return FreqUpdate::GatedQuality;
            }
        }
        // 4. α-β update. The level (`freq_mppb`) is a single EMA — fast alpha while converging, slow
        // after lock — projected one sample forward with the slope before correcting by the residual,
        // and the slope tracks the level's per-sample drift. A level-only EMA lags a frequency ramp
        // (temperature drift) by ≈2^ema_shift samples; the slope projection cancels that lag so the
        // disciplined output does not accrue a ramp-proportional phase error. Shifts clamped to
        // 0..=63 (i64 shift overflow).
        let level_shift = if self.is_locked() {
            self.config.ema_shift
        } else {
            self.config.ema_shift_fast
        }
        .min(63);
        let projected = self.freq_mppb + self.freq_slope_mppb;
        let resid = measured_mppb - projected;
        self.freq_mppb = projected + (resid >> level_shift);
        // Adapt the slope only once locked: while converging the level moves fast on its own, so a
        // slope estimate would just chase the start-up transient. Its EMA is far slower than the
        // level's (slope_ema_shift > ema_shift) to avoid differentiating PPS jitter, and clamped to a
        // physically plausible crystal ramp.
        if self.is_locked() && resid.abs() < self.config.slope_freeze_resid_mppb {
            let slope_shift = self.config.slope_ema_shift.min(63);
            self.freq_slope_mppb = (self.freq_slope_mppb + (resid >> slope_shift))
                .clamp(-self.config.slope_max_mppb, self.config.slope_max_mppb);
        }
        self.samples += 1;
        FreqUpdate::Applied
    }

    /// Call this when recovering to `Locked` from holdover/Irregular. Holds off the next
    /// `quarantine_samples` frequency-EMA updates (does not reset the EMA — for a short outage the
    /// past estimate is more trustworthy). Does nothing on the very first capture (samples==0), since
    /// there is no estimate to protect yet — don't delay the start-up capture.
    pub fn start_quarantine(&mut self) {
        if self.samples > 0 {
            self.quarantine = self.config.quarantine_samples;
        }
    }

    /// Whether we are in the recovery quarantine (for logging/debugging).
    pub fn in_quarantine(&self) -> bool {
        self.quarantine > 0
    }

    /// Associate a PPS edge with UTC and update the epoch.
    /// `capture_ns` = capture-timebase ns time (for err/now_from_capture_ns); `query_ns` =
    /// continuously-readable query-timebase time (for ticker/holdover). Both come from the same
    /// oscillator and share the same frequency offset.
    pub fn update_epoch(&mut self, capture_ns: u64, query_ns: u64, unix_ns: i64) {
        self.epoch_capture_ns = Some(capture_ns);
        self.epoch_query_ns = Some(query_ns);
        self.epoch_unix_ns = Some(unix_ns);
        self.last_query_ns = Some(query_ns);
    }

    /// Frequency-correct a local elapsed `d` (ns): true elapsed = d - d*ppb/1e9 = d - d*mppb/1e12.
    fn corrected(&self, d: i64) -> i64 {
        d - (d as i128 * self.freq_mppb as i128 / 1_000_000_000_000i128) as i64
    }

    /// Estimated frequency offset (ppb).
    pub fn freq_ppb(&self) -> i64 {
        let half = 500 * self.freq_mppb.signum();
        (self.freq_mppb + half) / 1000
    }

    /// Estimated frequency offset (milli-ppb) — the α-β *level*. For displaying ppm in the webapp
    /// and as the frequency for time-reading (`now_from_*`). Use [`predicted_freq_mppb`] for the
    /// output period.
    pub fn freq_mppb(&self) -> i64 {
        self.freq_mppb
    }

    /// Tracked frequency *slope* (milli-ppb per sample): how fast the crystal's offset is drifting,
    /// e.g. under a temperature ramp. 0 in steady state. Exposed for logging and tests.
    pub fn freq_slope_mppb(&self) -> i64 {
        self.freq_slope_mppb
    }

    /// Frequency estimate (milli-ppb) projected `pred_lead_samples` ahead — the value to feed the
    /// **output period**. A level-only estimate lags a temperature-driven frequency ramp, so the
    /// generated PPS runs at a stale frequency and its phase drifts proportionally to the ramp rate;
    /// projecting the tracked slope forward cancels that lag to first order (see the config docs and
    /// `freq_slope_mppb`). Reduces to `freq_mppb()` when the slope is 0 (steady state) or
    /// `pred_lead_samples` is 0.
    pub fn predicted_freq_mppb(&self) -> i64 {
        self.freq_mppb + self.freq_slope_mppb * self.config.pred_lead_samples
    }

    /// Whether enough samples have been seen to lock the frequency.
    pub fn is_locked(&self) -> bool {
        self.samples >= self.config.lock_samples.get()
    }

    /// **capture-timebase** local time → disciplined UTC (Unix ns). ns precision. For err
    /// measurement at the PPS edge.
    pub fn now_from_capture_ns(&self, capture_ns: u64) -> Option<i64> {
        let ep = self.epoch_capture_ns?;
        let eu = self.epoch_unix_ns?;
        Some(eu + self.corrected(capture_ns as i64 - ep as i64))
    }

    /// **query-timebase** local time → disciplined UTC. For continuous query (ticker). Sub-second
    /// resolution is the query clock's µs (e.g. Instant).
    pub fn now_from_query_ns(&self, query_ns: u64) -> Option<i64> {
        let ei = self.epoch_query_ns?;
        let eu = self.epoch_unix_ns?;
        Some(eu + self.corrected(query_ns as i64 - ei as i64))
    }

    /// Inverse: the **query-timebase** local time (ns) at which a given UTC arrives. Frequency
    /// corrected. For scheduling "do something at exact UTC time T" or a corrected wait.
    pub fn query_ns_for_unix_ns(&self, unix_ns: i64) -> Option<i64> {
        let ei = self.epoch_query_ns? as i64;
        let eu = self.epoch_unix_ns?;
        let dt = unix_ns - eu; // true elapsed (ns)
        // local elapsed = true / (1 - ppb/1e9) ≈ true + true*ppb/1e9. mppb is ppb*1000.
        let d = dt + (dt as i128 * self.freq_mppb as i128 / 1_000_000_000_000i128) as i64;
        Some(ei + d)
    }

    /// Inverse: the **capture-timebase** local time (ns) at which a given UTC arrives. The inverse of
    /// `now_from_capture_ns`. The core of `fire_at_utc(T)` — pass this value as the target tick to
    /// the PIO generate/compare SM to drive the pin exactly at UTC T. ns precision, since it is the
    /// same capture timebase as the capture.
    pub fn capture_ns_for_unix_ns(&self, unix_ns: i64) -> Option<i64> {
        let ep = self.epoch_capture_ns? as i64;
        let eu = self.epoch_unix_ns?;
        let dt = unix_ns - eu; // true elapsed (ns)
        let d = dt + (dt as i128 * self.freq_mppb as i128 / 1_000_000_000_000i128) as i64;
        Some(ep + d)
    }

    /// **Prediction residual** (ns): how far the disciplined clock's predicted UTC for a capture
    /// edge at `capture_ns` differs from that edge's known true UTC `unix_ns`, snapped to the
    /// nearest second (so a multi-second PPS gap doesn't read as an integer-second offset). This is
    /// the GPSDO's "how accurate is my disciplined time" self-check. `None` before the epoch is set.
    pub fn prediction_residual_ns(&self, capture_ns: u64, unix_ns: i64) -> Option<i64> {
        self.now_from_capture_ns(capture_ns)
            .map(|predicted| snap_to_second_ns(predicted - unix_ns))
    }

    /// **Fire residual** (ns): if [`capture_ns_for_unix_ns`](Self::capture_ns_for_unix_ns) were used
    /// to schedule an output edge at `unix_ns`, how far (snapped) it would land from the edge
    /// actually captured at `capture_ns` — i.e. the accuracy of the inverse (fire-at-UTC) path,
    /// measured against a real reference edge. `None` before the epoch is set.
    pub fn fire_residual_ns(&self, capture_ns: u64, unix_ns: i64) -> Option<i64> {
        self.capture_ns_for_unix_ns(unix_ns)
            .map(|tick| snap_to_second_ns(tick - capture_ns as i64))
    }

    /// The basis of a corrected delay: the local-clock ns needed to "wait `true_ns` of true time".
    /// The local clock is ppb fast/slow, so wait that much more/less. Layered over `Timer::after`
    /// it lets you wait with ±ppb accuracy rather than crystal tolerance
    /// (e.g. `after_micros(true_to_local_ns(us*1000)/1000)`).
    pub fn true_to_local_ns(&self, true_ns: i64) -> i64 {
        true_ns + (true_ns as i128 * self.freq_mppb as i128 / 1_000_000_000_000i128) as i64
    }

    /// Elapsed time since the last PPS discipline (holdover time, ns). Pass the query timebase.
    pub fn holdover_ns(&self, query_ns: u64) -> u64 {
        match self.last_query_ns {
            Some(t) => query_ns.saturating_sub(t),
            None => 0,
        }
    }
}

/// Result of [`Gnssdo::on_pps`].
#[derive(Debug)]
pub struct GnssdoStep {
    /// How the tracker classified this edge.
    pub event: PpsEvent,
    /// The frequency-discipline outcome, or `None` if discipline was not attempted this edge (the
    /// edge was not `Locked`, or there was no valid interval).
    pub freq: Option<FreqUpdate>,
    /// Whether this edge was a holdover recovery (a non-`Locked` → `Locked` transition).
    pub recovered: bool,
}

/// Turn-key GNSS-disciplined clock — the **easy tier**: a [`PpsTracker`] + a [`DisciplinedClock`]
/// wired with the recommended default policy (discipline the frequency **only on `Locked` edges**
/// and quarantine the estimate for a few edges after recovering from a gap). For a different policy,
/// drive [`PpsTracker`] and [`DisciplinedClock`] yourself (the fine tier) — this bundles one good
/// default.
///
/// It owns no I/O: feed it PPS intervals via [`on_pps`](Self::on_pps) (from any capture source — the
/// RP2040 PIO via [`rp-pps`](https://docs.rs/rp-pps), a timer input-capture, `/dev/pps`, …) and
/// NMEA-derived UTC seconds via [`on_utc`](Self::on_utc); read back disciplined UTC, the frequency
/// offset, and holdover.
#[derive(Debug, Default)]
pub struct Gnssdo {
    tracker: PpsTracker,
    clock: DisciplinedClock,
    last_pps_locked: bool,
}

impl Gnssdo {
    /// Create with the default tracker/clock tuning and the recommended policy.
    pub const fn new() -> Self {
        Self {
            tracker: PpsTracker::new(),
            clock: DisciplinedClock::new(),
            last_pps_locked: false,
        }
    }

    /// Create with given tracker / clock configurations. `const fn` so it can init a `static`.
    pub const fn with_config(tracker: PpsTrackerConfig, clock: DisciplinedClockConfig) -> Self {
        Self {
            tracker: PpsTracker::with_config(tracker),
            clock: DisciplinedClock::with_config(clock),
            last_pps_locked: false,
        }
    }

    /// Feed one PPS edge. `capture_time_us` is the edge's running capture-timebase time (µs; the
    /// tracker uses it to spot missed/irregular edges) and `interval_ns` is the precise interval
    /// since the previous edge (the frequency estimate's input). The edge is classified and, **only
    /// when it is `Locked`**, the frequency is disciplined — quarantining the estimate on a recovery.
    ///
    /// `capture_time_us` and `interval_ns` are separate inputs from the same edge stream; pass
    /// consistent values (e.g. both from `rp-pps`'s `PpsEdgeTimeline`).
    pub fn on_pps(&mut self, capture_time_us: u64, interval_ns: i64) -> GnssdoStep {
        let event = self.tracker.record(capture_time_us);
        let locked = matches!(event, PpsEvent::Locked { .. });
        let recovered = locked && !self.last_pps_locked;
        self.last_pps_locked = locked;
        let freq = if interval_ns > 0 && locked {
            if recovered {
                self.clock.start_quarantine();
            }
            Some(self.clock.update_freq(interval_ns))
        } else {
            None
        };
        GnssdoStep {
            event,
            freq,
            recovered,
        }
    }

    /// Establish/refresh the UTC epoch by pairing a PPS edge with its UTC second. A thin passthrough
    /// to [`DisciplinedClock::update_epoch`]; NMEA parsing and the PPS↔NMEA pairing stay the caller's
    /// (the `rp-pps` crate provides them).
    pub fn on_utc(&mut self, capture_ns: u64, query_ns: u64, unix_ns: i64) {
        self.clock.update_epoch(capture_ns, query_ns, unix_ns);
    }

    /// Disciplined UTC (ns) from a continuously-readable query-timebase local time.
    pub fn now_from_query_ns(&self, query_ns: u64) -> Option<i64> {
        self.clock.now_from_query_ns(query_ns)
    }

    /// Estimated crystal frequency offset (ppb).
    pub fn freq_ppb(&self) -> i64 {
        self.clock.freq_ppb()
    }

    /// Time since the last epoch update (ns) — the current holdover duration.
    pub fn holdover_ns(&self, query_ns: u64) -> u64 {
        self.clock.holdover_ns(query_ns)
    }

    /// Whether the **frequency estimate** has locked (enough samples seen). Distinct from
    /// [`last_pps_locked`](Self::last_pps_locked) (which is about the most recent edge's class).
    pub fn frequency_locked(&self) -> bool {
        self.clock.is_locked()
    }

    /// Whether the most recent edge was classified `Locked` by the tracker.
    pub fn last_pps_locked(&self) -> bool {
        self.last_pps_locked
    }

    /// The inner [`DisciplinedClock`], for fine-tier queries — e.g.
    /// [`prediction_residual_ns`](DisciplinedClock::prediction_residual_ns),
    /// [`fire_residual_ns`](DisciplinedClock::fire_residual_ns),
    /// [`capture_ns_for_unix_ns`](DisciplinedClock::capture_ns_for_unix_ns).
    pub fn clock(&self) -> &DisciplinedClock {
        &self.clock
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::num::{NonZeroU32, NonZeroU64};

    #[test]
    fn custom_config_changes_lock_threshold() {
        // Lowering lock_samples to 3 locks after 3 samples (unlike the default of 8).
        let cfg = DisciplinedClockConfig {
            lock_samples: NonZeroU32::new(3).unwrap(),
            ..DisciplinedClockConfig::DEFAULT
        };
        let mut c = DisciplinedClock::with_config(cfg);
        c.update_freq(1_000_002_500);
        c.update_freq(1_000_002_500);
        assert!(!c.is_locked());
        c.update_freq(1_000_002_500);
        assert!(c.is_locked());
    }

    #[test]
    fn custom_converge_gate_admits_wider_outlier() {
        // Widening converge_gate_ns admits, while converging, a deviation the default (±100µs) rejects.
        let cfg = DisciplinedClockConfig {
            converge_gate_ns: NonZeroU64::new(500_000).unwrap(),
            ..DisciplinedClockConfig::DEFAULT
        };
        let mut c = DisciplinedClock::with_config(cfg);
        c.update_freq(1_000_002_500); // establish the reference
        assert_eq!(c.update_freq(1_000_000_000 + 200_000), FreqUpdate::Applied);
    }

    #[test]
    fn freq_converges_to_constant_offset() {
        let mut c = DisciplinedClock::new();
        for _ in 0..40 {
            c.update_freq(1_000_002_500); // +2500 ppb (= +2.5 ppm)
        }
        assert_eq!(c.freq_ppb(), 2500);
        assert!(c.is_locked());
    }

    #[test]
    fn freq_ema_smooths_jitter() {
        let mut c = DisciplinedClock::new();
        // Input swinging ±16ns around 2500 → after smoothing ≈ 2500.
        for i in 0..200 {
            let j = if i % 2 == 0 { 16 } else { -16 };
            c.update_freq(1_000_000_000 + 2500 + j);
        }
        assert!((c.freq_ppb() - 2500).abs() <= 20, "freq={}", c.freq_ppb());
    }

    #[test]
    fn predicted_freq_leads_a_frequency_ramp() {
        // 温度ドリフト = 水晶周波数のランプ。単一 EMA はランプを時定数分だけ遅れて追うので、
        // その値を出力周期に使うと位相がランプ速度に比例してずれる (実機 offset② の正体)。
        // α-β は傾き (slope) を推定して 1 サンプル先を予測 → ランプに対し定常遅れ ~0。
        let mut c = DisciplinedClock::new();
        let f0 = 3_000_000i64; // 始点 3000 ppb (mppb)
        let r = 2_000i64; // +2000 mppb/sample = +2 ppb/s のランプ (1000 の倍数で test 丸めを回避)
        let mut n = 0i64;
        for _ in 0..80 {
            let measured_mppb = f0 + r * n;
            c.update_freq(1_000_000_000 + measured_mppb / 1000);
            n += 1;
        }
        // 次サンプル (n=80) の真の周波数
        let true_next = f0 + r * n;
        // α-β の 1 サンプル先予測がランプを追えている (定常遅れ ~0)。
        // 単一 EMA だと ~31*r=62000 mppb 遅れて落ちる。
        let pred = c.predicted_freq_mppb();
        assert!(
            (pred - true_next).abs() < 10_000,
            "pred={} true_next={} lag={}",
            pred,
            true_next,
            pred - true_next
        );
        // 傾き推定がランプ速度に収束。
        assert!(
            (c.freq_slope_mppb() - r).abs() < r / 2,
            "slope={} expected~{}",
            c.freq_slope_mppb(),
            r
        );
    }

    #[test]
    fn alpha_beta_cuts_ramp_phase_error_vs_single_ema() {
        // offset② の核心: 周波数ランプ(温度ドリフト)に対し、出力周期に入れる周波数が真値からずれた分は
        // 位相として積算される(PLL が吸収すべき量)。単一 EMA は ramp を時定数分遅れて追うので大きく積算し、
        // α-β は slope を先回りするので桁で小さい。両者で「累積位相のピーク」を比べ、α-β が 1/5 未満を保証。
        let f0 = 3_000_000i64;
        let r = 2_000i64; // 2 ppb/s ランプを 200s, その後一定 200s
        let truef = |n: i64| if n < 200 { f0 + r * n } else { f0 + r * 199 };
        let peak_phase = |slope_max: i64| -> i64 {
            let cfg = DisciplinedClockConfig {
                slope_max_mppb: slope_max, // 0 = slope 凍結 = 真の単一 EMA ベースライン
                ..DisciplinedClockConfig::DEFAULT
            };
            let mut c = DisciplinedClock::with_config(cfg);
            let (mut phase_ns, mut peak) = (0i64, 0i64);
            for n in 0..400i64 {
                c.update_freq(1_000_000_000 + truef(n) / 1000);
                let avg_true = (truef(n) + truef(n + 1)) / 2;
                phase_ns += (c.predicted_freq_mppb() - avg_true) / 1000;
                if phase_ns.abs() > peak.abs() {
                    peak = phase_ns;
                }
            }
            peak.abs()
        };
        let ema = peak_phase(0);
        let ab = peak_phase(DisciplinedClockConfig::DEFAULT.slope_max_mppb);
        // 単一 EMA は ~12µs、α-β は ~1.3µs。少なくとも 5 倍の改善を回帰として固定。
        assert!(ema > 8_000, "single-EMA ramp phase should be large, got {ema}ns");
        assert!(
            ab * 5 < ema,
            "α-β should cut ramp phase error >5x: ema={ema}ns ab={ab}ns"
        );
    }

    #[test]
    fn slope_freezes_on_large_residual_shock() {
        // 緩いランプ (dev = 3000 + 2n ns = +2 ppb/sample) で slope を立ててロック。
        let mut c = DisciplinedClock::new();
        for n in 0..40i64 {
            c.update_freq(1_000_000_000 + 3000 + 2 * n);
        }
        let slope = c.freq_slope_mppb();
        assert!(slope != 0, "ramp should establish a nonzero slope");
        // 残差 ~3µs のサンプル (ショック相当)。±5µs ゲートは通る (level 更新) が、slope_freeze_resid(2µs)
        // を超えるので slope は凍結される (spike も stale FF も防ぐ)。
        let level_ppb = c.freq_ppb();
        c.update_freq(1_000_000_000 + level_ppb + 3000);
        assert_eq!(
            c.freq_slope_mppb(),
            slope,
            "slope must freeze on a large (shock) residual"
        );
    }

    #[test]
    fn quarantine_freezes_slope() {
        // 復帰直後の検疫中は level だけでなく slope 更新も止める(復帰直後の怪しい PPS で傾きを学習しない)。
        let mut c = DisciplinedClock::new();
        // ランプ (dev = 3000 + 2n ns) で slope を立ててロックさせる。
        for n in 0..40i64 {
            c.update_freq(1_000_000_000 + 3000 + 2 * n);
        }
        let slope_before = c.freq_slope_mppb();
        c.start_quarantine();
        for _ in 0..c.config().quarantine_samples {
            assert_eq!(c.update_freq(1_000_010_000), FreqUpdate::Quarantined);
        }
        assert_eq!(c.freq_slope_mppb(), slope_before, "slope must not move in quarantine");
    }

    #[test]
    fn slope_is_zero_for_constant_offset_so_pred_equals_level() {
        // 定数オフセットでは傾き 0 → 予測 = level = freq_mppb (単一 EMA と等価)。
        let mut c = DisciplinedClock::new();
        for _ in 0..60 {
            c.update_freq(1_000_002_500); // +2500 ppb 一定
        }
        assert_eq!(c.freq_ppb(), 2500);
        assert_eq!(c.freq_slope_mppb(), 0);
        assert_eq!(c.predicted_freq_mppb(), c.freq_mppb());
    }

    #[test]
    fn single_outlier_does_not_move_slope() {
        // 単発の外れ値は残差ゲートで弾かれ、傾き推定を汚さない (微分器化の防止)。
        let mut c = DisciplinedClock::new();
        for _ in 0..20 {
            c.update_freq(1_000_002_500); // lock, slope=0
        }
        let slope_before = c.freq_slope_mppb();
        c.update_freq(1_000_000_000 + 2500 + 10_000); // +10µs マルチパススパイク → gate
        assert_eq!(c.freq_slope_mppb(), slope_before);
    }

    #[test]
    fn outliers_ignored() {
        let mut c = DisciplinedClock::new();
        c.update_freq(1_000_002_500);
        c.update_freq(500_000_000); // wrap-derived outlier → ignored
        c.update_freq(2_000_000_000); // same
        assert_eq!(c.freq_ppb(), 2500);
        assert_eq!(c.samples, 1);
    }

    #[test]
    fn update_freq_returns_applied_or_gated() {
        let mut c = DisciplinedClock::new();
        assert_eq!(c.update_freq(1_000_002_500), FreqUpdate::Applied);
        assert_eq!(c.update_freq(500_000_000), FreqUpdate::GatedSane); // emergency stop
    }

    #[test]
    fn pre_lock_converge_gate_rejects_midsize_outlier() {
        let mut c = DisciplinedClock::new();
        c.update_freq(1_000_002_500); // first sample establishes the reference (+2500ppb)
        // While unlocked, medium multipath within ±1ms but beyond ±100µs is rejected.
        assert_eq!(
            c.update_freq(1_000_000_000 + 200_000),
            FreqUpdate::GatedQuality
        );
        assert_eq!(c.samples, 1);
        assert_eq!(c.freq_ppb(), 2500); // not contaminated
    }

    #[test]
    fn post_lock_residual_gate_rejects_multipath() {
        let mut c = DisciplinedClock::new();
        for _ in 0..12 {
            c.update_freq(1_000_002_500); // lock at +2500ppb
        }
        assert!(c.is_locked());
        // After lock, a single sample +10µs off the EMA (multipath) is rejected by the residual gate.
        assert_eq!(
            c.update_freq(1_000_000_000 + 2500 + 10_000),
            FreqUpdate::GatedQuality
        );
        assert_eq!(c.freq_ppb(), 2500); // the EMA does not move
        // Near the EMA (±tens of ns jitter) passes.
        assert_eq!(c.update_freq(1_000_002_500 + 16), FreqUpdate::Applied);
    }

    #[test]
    fn quarantine_holds_updates_after_recovery() {
        let mut c = DisciplinedClock::new();
        for _ in 0..12 {
            c.update_freq(1_000_002_500); // lock (+2500ppb)
        }
        c.start_quarantine();
        assert!(c.in_quarantine());
        // The first 5 samples after recovery do not update the EMA (even if within range).
        for _ in 0..5 {
            assert_eq!(c.update_freq(1_000_005_000), FreqUpdate::Quarantined); // suspect +5000ppb PPS
        }
        assert_eq!(c.freq_ppb(), 2500); // quarantine protected the EMA
        assert!(!c.in_quarantine());
        // After quarantine ends it updates as usual.
        assert_eq!(c.update_freq(1_000_002_500), FreqUpdate::Applied);
    }

    #[test]
    fn quarantine_noop_on_cold_boot() {
        let mut c = DisciplinedClock::new();
        c.start_quarantine(); // no estimate yet → no quarantine
        assert!(!c.in_quarantine());
        // The first capture is accepted immediately (start-up capture is not delayed).
        assert_eq!(c.update_freq(1_000_002_500), FreqUpdate::Applied);
        assert_eq!(c.freq_ppb(), 2500);
    }

    #[test]
    fn fast_alpha_converges_within_few_samples() {
        let mut c = DisciplinedClock::new();
        // With the fast unlocked alpha=1/8, it approaches the steady offset within a few samples.
        for _ in 0..DisciplinedClockConfig::DEFAULT.lock_samples.get() {
            c.update_freq(1_000_003_000); // +3000ppb
        }
        // It sticks to 3000 on the first sample, so it's already ≈3000 by the time it locks.
        assert!((c.freq_ppb() - 3000).abs() <= 50, "freq={}", c.freq_ppb());
    }

    #[test]
    fn not_locked_until_enough_samples() {
        let mut c = DisciplinedClock::new();
        for _ in 0..(DisciplinedClockConfig::DEFAULT.lock_samples.get() - 1) {
            c.update_freq(1_000_002_500);
        }
        assert!(!c.is_locked());
        c.update_freq(1_000_002_500);
        assert!(c.is_locked());
    }

    #[test]
    fn now_without_correction() {
        let mut c = DisciplinedClock::new();
        c.update_epoch(1_000_000_000, 1_000_000_000, 5_000_000_000_000);
        // freq=0 → no correction. 0.5s later.
        assert_eq!(
            c.now_from_capture_ns(1_500_000_000),
            Some(5_000_500_000_000)
        );
    }

    #[test]
    fn now_applies_freq_correction_during_holdover() {
        let mut c = DisciplinedClock::new();
        // +100,000 ppb = +100 ppm (exaggerated) to make the correction visible (kept within the ±1ms filter).
        for _ in 0..40 {
            c.update_freq(1_000_000_000 + 100_000);
        }
        assert_eq!(c.freq_ppb(), 100_000);
        c.update_epoch(0, 0, 0);
        // local elapsed 1e9 ns. true elapsed = 1e9 - 1e9*1e5/1e12 = 1e9 - 1e5.
        assert_eq!(
            c.now_from_capture_ns(1_000_000_000),
            Some(1_000_000_000 - 100_000)
        );
    }

    #[test]
    fn holdover_counts_since_last_epoch() {
        let mut c = DisciplinedClock::new();
        c.update_epoch(1_000_000_000, 1_000_000_000, 0);
        assert_eq!(c.holdover_ns(1_000_000_000), 0);
        assert_eq!(c.holdover_ns(4_000_000_000), 3_000_000_000); // 3s holdover
    }

    #[test]
    fn true_to_local_applies_offset() {
        let mut c = DisciplinedClock::new();
        // freq=0 → no correction (identity).
        assert_eq!(c.true_to_local_ns(1_000_000_000), 1_000_000_000);
        for _ in 0..40 {
            c.update_freq(1_000_000_000 + 100_000); // +100 ppm
        }
        // The local clock is fast, so to wait a true 1s you wait +100µs more on the local clock.
        assert_eq!(c.true_to_local_ns(1_000_000_000), 1_000_000_000 + 100_000);
    }

    #[test]
    fn snap_recovers_subsecond_residual() {
        // Normal small residuals pass through unchanged.
        assert_eq!(snap_to_second_ns(360), 360);
        assert_eq!(snap_to_second_ns(-41), -41);
        // Recover the residual buried in a 1-second / 25-second integer offset.
        assert_eq!(snap_to_second_ns(1_000_000_041), 41);
        assert_eq!(snap_to_second_ns(25_000_000_360), 360);
        assert_eq!(snap_to_second_ns(-1_000_000_041), -41);
        // Just before the second boundary (100ns before 1s) is -100ns.
        assert_eq!(snap_to_second_ns(999_999_900), -100);
    }

    #[test]
    fn now_is_none_before_epoch() {
        let c = DisciplinedClock::new();
        assert_eq!(c.now_from_capture_ns(123), None);
    }

    #[test]
    fn local_for_unix_roundtrips_with_now() {
        let mut c = DisciplinedClock::new();
        for _ in 0..40 {
            c.update_freq(1_000_000_000 + 3000); // +3 ppm (a realistic crystal offset)
        }
        assert_eq!(c.freq_ppb(), 3000);
        c.update_epoch(1_000_000_000, 1_000_000_000, 5_000_000_000_000);
        // Find the local time at which UTC 3 seconds later arrives; querying now at that local time
        // returns the original UTC (query timebase).
        let target = 5_000_000_000_000 + 3_000_000_000;
        let local = c.query_ns_for_unix_ns(target).unwrap();
        let back = c.now_from_query_ns(local as u64).unwrap();
        assert!(
            (back - target).abs() <= 2,
            "roundtrip off by {}",
            back - target
        );
        // If the correction works, local elapsed > true elapsed (the fast crystal runs ahead): +3ppm×3s ≈ +9µs.
        assert!(local > 1_000_000_000 + 3_000_000_000);
    }

    #[test]
    fn pio_local_for_unix_roundtrips_with_now() {
        // Core of fire_at_utc: the UTC → capture-tick inverse round-trips with now_from_capture_ns (capture tick → UTC).
        let mut c = DisciplinedClock::new();
        for _ in 0..40 {
            c.update_freq(1_000_000_000 + 3000); // +3 ppm
        }
        c.update_epoch(1_000_000_000, 1_000_000_000, 5_000_000_000_000);
        let target = 5_000_000_000_000 + 3_000_000_000; // UTC 3 seconds later
        let tick = c.capture_ns_for_unix_ns(target).unwrap(); // drive the pin at this capture tick for UTC=target
        let back = c.now_from_capture_ns(tick as u64).unwrap();
        assert!(
            (back - target).abs() <= 2,
            "roundtrip off by {}",
            back - target
        );
        // The crystal is +3ppm fast, so for UTC 3s ahead the capture tick is ahead of the true elapsed (+9µs).
        assert!(tick > 1_000_000_000 + 3_000_000_000);
    }

    #[test]
    fn pio_local_for_unix_none_before_epoch() {
        let c = DisciplinedClock::new();
        assert_eq!(c.capture_ns_for_unix_ns(5_000_000_000_000), None);
    }

    #[test]
    fn pio_local_for_unix_no_freq_offset_is_identity_shift() {
        // With freq=0, capture tick = epoch_capture + (unix - epoch_unix) (no correction).
        let mut c = DisciplinedClock::new();
        c.update_epoch(1_000_000_000, 1_000_000_000, 5_000_000_000_000);
        assert_eq!(
            c.capture_ns_for_unix_ns(5_000_000_500_000),
            Some(1_000_000_000 + 500_000)
        );
    }

    #[test]
    fn residuals_are_zero_when_edge_matches_utc() {
        let mut c = DisciplinedClock::new();
        c.update_epoch(1_000_000_000, 1_000_000_000, 5_000_000_000_000);
        // An edge 1 s after the epoch whose true UTC is exactly 1 s after the epoch UTC (freq 0).
        let cap = 1_000_000_000 + 1_000_000_000;
        let utc = 5_000_000_000_000 + 1_000_000_000;
        assert_eq!(c.prediction_residual_ns(cap, utc), Some(0));
        assert_eq!(c.fire_residual_ns(cap, utc), Some(0));
    }

    #[test]
    fn residuals_reflect_a_late_edge() {
        let mut c = DisciplinedClock::new();
        c.update_epoch(1_000_000_000, 1_000_000_000, 5_000_000_000_000);
        let utc = 5_000_000_000_000 + 1_000_000_000;
        let cap = 1_000_000_000 + 1_000_000_000 + 500; // edge 500 ns late vs the UTC second
        // predicted UTC is 500 ns ahead of true UTC; the scheduled tick is 500 ns before the edge.
        assert_eq!(c.prediction_residual_ns(cap, utc), Some(500));
        assert_eq!(c.fire_residual_ns(cap, utc), Some(-500));
    }

    #[test]
    fn residuals_none_before_epoch() {
        let c = DisciplinedClock::new();
        assert_eq!(c.prediction_residual_ns(123, 456), None);
        assert_eq!(c.fire_residual_ns(123, 456), None);
    }

    #[test]
    fn gnssdo_disciplines_only_on_locked() {
        let mut g = Gnssdo::new();
        // First edge: classified First, no discipline.
        let s0 = g.on_pps(0, 0);
        assert_eq!(s0.event, PpsEvent::First);
        assert!(s0.freq.is_none());
        assert!(!g.frequency_locked() && !g.last_pps_locked());

        // A locked 1 Hz edge with a precise interval → disciplined; first lock counts as recovery.
        let s1 = g.on_pps(1_000_000, 1_000_000_000);
        assert!(matches!(s1.event, PpsEvent::Locked { .. }));
        assert_eq!(s1.freq, Some(FreqUpdate::Applied));
        assert!(s1.recovered);
        assert!(g.last_pps_locked());

        // Steady locked edge: disciplined, not a recovery.
        let s2 = g.on_pps(2_000_000, 1_000_000_000);
        assert!(matches!(s2.event, PpsEvent::Locked { .. }));
        assert!(!s2.recovered);
        assert_eq!(s2.freq, Some(FreqUpdate::Applied));
    }

    #[test]
    fn gnssdo_skips_discipline_on_irregular() {
        let mut g = Gnssdo::new();
        g.on_pps(0, 0);
        g.on_pps(1_000_000, 1_000_000_000);
        // A ~2 s gap → Irregular → no frequency discipline this edge.
        let s = g.on_pps(3_000_000, 2_000_000_000);
        assert!(matches!(s.event, PpsEvent::Irregular { .. }));
        assert!(s.freq.is_none());
    }

    #[test]
    fn gnssdo_epoch_and_fine_tier() {
        let mut g = Gnssdo::new();
        assert_eq!(g.now_from_query_ns(123), None); // no epoch yet
        g.on_utc(1_000_000_000, 1_000_000_000, 5_000_000_000_000);
        assert_eq!(
            g.now_from_query_ns(1_000_000_000 + 500),
            Some(5_000_000_000_000 + 500)
        );
        // the fine tier is reachable via clock()
        assert_eq!(
            g.clock().capture_ns_for_unix_ns(5_000_000_000_000),
            Some(1_000_000_000)
        );
    }
}
