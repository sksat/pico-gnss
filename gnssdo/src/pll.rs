//! Output-phase discipline for a GPSDO's generated 1PPS — a type-II phase-locked loop (PLL).
//!
//! Where [`DisciplinedClock`](crate::DisciplinedClock) disciplines the device's *internal* time
//! from the incoming reference PPS, [`PhaseLockLoop`] disciplines a *generated* output PPS so its
//! edge phase-locks to the reference. It is the control counterpart: integer-only, HAL-agnostic,
//! host-testable. It consumes a measured phase error (ns, e.g. from a hardware loopback capture)
//! and returns a frequency trim (milli-ppb) and an immediate phase correction (ns) — it does
//! **not** produce the final PIO period word (that quantization/dither is the I/O layer's job;
//! see `rp-pps`'s `OutputPeriodDither`).
//!
//! The loop is type-II (P + I) with an optional derivative term and a **Smith predictor** that
//! subtracts the in-flight (not-yet-observed) correction so a higher proportional gain stays
//! stable. Lock detection and single-sample outlier rejection guard against weak-signal spikes.
//! Defaults ([`PhaseLockLoopConfig::DEFAULT`]) are the values measured to give σ≈35 ns on an
//! RP2040 PIO loopback (see the firmware's NOTES); they are configurable rather than fixed consts.

/// Safety clamp on a single edge's phase correction (ns). A correction larger than this is a
/// glitch; never move the output by more than 100 ms in one step. Not a tuning knob.
const CORR_CLAMP_NS: i64 = 100_000_000;

/// Which control terms are active. `PidSmith` is production; the others exist to compare terms
/// (the firmware's experiment harness switches between them).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LoopMode {
    /// Proportional only.
    P,
    /// Proportional + derivative.
    Pd,
    /// Proportional + integral (type-II).
    Pi,
    /// P + I + D.
    Pid,
    /// P + I + D with the Smith predictor — the production configuration.
    #[default]
    PidSmith,
}

impl LoopMode {
    fn use_i(self) -> bool {
        matches!(self, LoopMode::Pi | LoopMode::Pid | LoopMode::PidSmith)
    }
    fn use_d(self) -> bool {
        matches!(self, LoopMode::Pd | LoopMode::Pid | LoopMode::PidSmith)
    }
    fn smith(self) -> bool {
        matches!(self, LoopMode::PidSmith)
    }
    /// Inverse proportional gain. Smith-compensated loops tolerate a higher gain (1/8); without
    /// the predictor the loop delay forces 1/16 to stay stable.
    fn kp_inv(self) -> i64 {
        if self.smith() { 8 } else { 16 }
    }
}

/// Tuning for [`PhaseLockLoop`]. `Default` (= [`PhaseLockLoopConfig::DEFAULT`]) is the σ≈35 ns
/// production tuning. The right values depend on the loop delay, the capture resolution, and the
/// reference PPS quality, so they are configurable.
///
/// (`Debug`/`Default` is the minimum [`PhaseLockLoop`]'s derive requires.)
#[derive(Debug)]
pub struct PhaseLockLoopConfig {
    /// Active control terms (default [`LoopMode::PidSmith`]).
    pub mode: LoopMode,
    /// Phase deadband (ns): below this the P term is skipped. Default 0 — with the Smith predictor
    /// the P gain is gentle enough to leave on across the whole range (avoids an undamped band).
    pub deadband_ns: i64,
    /// Lock window (ns): an edge whose phase is within this counts toward lock. Default 1 µs.
    pub lock_ns: i64,
    /// Consecutive in-window edges required to declare lock. Default 5.
    pub lock_hold: u32,
    /// Post-lock outlier threshold (ns): a single edge beyond this while locked is treated as a
    /// weak-signal spike and held (no correction). Default 3 µs.
    pub outlier_ns: i64,
    /// Max consecutive outliers to reject before accepting one as a real disturbance (re-lock).
    /// Default 12.
    pub outlier_max: u32,
    /// Integral denominator: trim += −pred·1000 / `i_den` per locked edge (milli-ppb). Larger =
    /// slower integration. Default 128 (loop natural period ≈ 2π√128 edges).
    pub i_den: i64,
    /// Clamp on the integrated frequency trim (milli-ppb). Default 3 000 000 (= ±3000 ppb).
    pub trim_max_mppb: i64,
    /// Derivative denominator: d_corr = (pred − last_pred) / `d_den` (ns). Default 4.
    pub d_den: i64,
}

impl PhaseLockLoopConfig {
    /// The measured σ≈35 ns production tuning. `const` so [`PhaseLockLoop::new`] is `const`.
    pub const DEFAULT: Self = Self {
        mode: LoopMode::PidSmith,
        deadband_ns: 0,
        lock_ns: 1_000,
        lock_hold: 5,
        outlier_ns: 3_000,
        outlier_max: 12,
        i_den: 128,
        trim_max_mppb: 3_000_000,
        d_den: 4,
    };
}

impl Default for PhaseLockLoopConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Result of [`PhaseLockLoop::update`]. The caller forms the output period from
/// `freq_trim_mppb` (added to the crystal estimate) and `phase_corr_ns`; the rest is for logging.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PhaseLockLoopUpdate {
    /// Whether this edge was applied (false if invalid or rejected as an outlier).
    pub applied: bool,
    /// Lock state used for this edge's control decisions (lock attained on a prior edge).
    pub locked: bool,
    /// Whether this edge was held as a post-lock outlier.
    pub rejected_outlier: bool,
    /// Integrated frequency trim (milli-ppb) to add to the crystal's estimated offset.
    pub freq_trim_mppb: i64,
    /// Immediate phase correction (ns) to subtract from this period (= `p_corr_ns + d_corr_ns`).
    pub phase_corr_ns: i64,
    /// Proportional component of `phase_corr_ns` (ns).
    pub p_corr_ns: i64,
    /// Derivative component of `phase_corr_ns` (ns).
    pub d_corr_ns: i64,
    /// The Smith-predicted phase used as the control input this edge (ns).
    pub predicted_phase_ns: i64,
}

/// A type-II output-phase PLL. Feed it a measured phase error each output edge; it integrates a
/// frequency trim and emits an immediate phase correction. See the [module docs](self).
#[derive(Debug, Default)]
pub struct PhaseLockLoop {
    config: PhaseLockLoopConfig,
    lock_cnt: u32,
    reject_cnt: u32,
    trim_mppb: i64,
    last_pred: i64, // previous edge's predicted phase (for the derivative term)
    last_pd: i64, // previous edge's p+d correction (the in-flight amount the Smith predictor removes)
}

impl PhaseLockLoop {
    /// Create with the production defaults ([`PhaseLockLoopConfig::DEFAULT`]).
    pub const fn new() -> Self {
        Self::with_config(PhaseLockLoopConfig::DEFAULT)
    }

    /// Create with a given configuration. `const fn` so it can initialize a `static`.
    pub const fn with_config(config: PhaseLockLoopConfig) -> Self {
        Self {
            config,
            lock_cnt: 0,
            reject_cnt: 0,
            trim_mppb: 0,
            last_pred: 0,
            last_pd: 0,
        }
    }

    /// Current configuration.
    pub fn config(&self) -> &PhaseLockLoopConfig {
        &self.config
    }

    /// Switch the active control terms (the firmware's experiment harness uses this; production
    /// leaves it at the default [`LoopMode::PidSmith`]).
    pub fn set_mode(&mut self, mode: LoopMode) {
        self.config.mode = mode;
    }

    /// Whether the loop is currently locked.
    pub fn is_locked(&self) -> bool {
        self.lock_cnt >= self.config.lock_hold
    }

    /// Current integrated frequency trim (milli-ppb).
    pub fn freq_trim_mppb(&self) -> i64 {
        self.trim_mppb
    }

    /// Process one output edge. `phase_err_ns` is the measured output-vs-reference phase (ns);
    /// `valid` is whether this sample is trustworthy (reference fresh, interval sane) — when false
    /// the loop holds its trim and lock state and emits no correction (holdover).
    pub fn update(&mut self, phase_err_ns: i64, valid: bool) -> PhaseLockLoopUpdate {
        let ctrl = phase_err_ns;
        // Smith predictor: control on the phase minus the still-in-flight correction.
        let pred = if self.config.mode.smith() {
            ctrl - self.last_pd
        } else {
            ctrl
        };
        let locked = self.is_locked();
        let mut p_corr = 0;
        let mut d_corr = 0;
        let mut applied = false;
        let mut rejected_outlier = false;

        if valid {
            if locked
                && ctrl.abs() > self.config.outlier_ns
                && self.reject_cnt < self.config.outlier_max
            {
                // Post-lock single-sample spike: hold (don't move the output).
                self.reject_cnt += 1;
                rejected_outlier = true;
            } else {
                self.reject_cnt = 0;
                applied = true;
                // I term: integrate the predicted phase into the frequency trim (milli-ppb) while
                // locked, driving the steady-state offset to zero. Reset when the I term is off.
                if self.config.mode.use_i() {
                    if locked {
                        self.trim_mppb = (self.trim_mppb - pred * 1000 / self.config.i_den)
                            .clamp(-self.config.trim_max_mppb, self.config.trim_max_mppb);
                    }
                } else {
                    self.trim_mppb = 0;
                }
                // P term.
                if pred.abs() > self.config.deadband_ns {
                    p_corr =
                        (pred / self.config.mode.kp_inv()).clamp(-CORR_CLAMP_NS, CORR_CLAMP_NS);
                }
                // D term (only while locked).
                if self.config.mode.use_d() && locked {
                    d_corr = ((pred - self.last_pred) / self.config.d_den)
                        .clamp(-CORR_CLAMP_NS, CORR_CLAMP_NS);
                }
                self.lock_cnt = if ctrl.abs() < self.config.lock_ns {
                    (self.lock_cnt + 1).min(self.config.lock_hold)
                } else {
                    0
                };
            }
        }
        // Invalid (reference lost, etc.): freq trim + lock state are held; no correction this edge.

        self.last_pred = pred;
        self.last_pd = p_corr + d_corr;
        PhaseLockLoopUpdate {
            applied,
            locked,
            rejected_outlier,
            freq_trim_mppb: self.trim_mppb,
            phase_corr_ns: p_corr + d_corr,
            p_corr_ns: p_corr,
            d_corr_ns: d_corr,
            predicted_phase_ns: pred,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drive `n` valid edges at a fixed phase; return the last update.
    fn run(pll: &mut PhaseLockLoop, phase: i64, n: usize) -> PhaseLockLoopUpdate {
        let mut u = pll.update(phase, true);
        for _ in 1..n {
            u = pll.update(phase, true);
        }
        u
    }

    #[test]
    fn default_is_production_pid_smith() {
        let p = PhaseLockLoop::new();
        assert_eq!(p.config().mode, LoopMode::PidSmith);
        assert_eq!(p.config().lock_ns, 1_000);
        assert_eq!(p.config().i_den, 128);
    }

    #[test]
    fn p_term_first_edge_uses_kp_inv_8_and_no_smith_history() {
        let mut p = PhaseLockLoop::new();
        // First edge: last_pd=0 so pred=ctrl; PidSmith kp_inv=8; not locked so no I/D.
        let u = p.update(800, true);
        assert_eq!(u.predicted_phase_ns, 800);
        assert_eq!(u.p_corr_ns, 800 / 8);
        assert_eq!(u.d_corr_ns, 0);
        assert_eq!(u.freq_trim_mppb, 0);
        assert!(!u.locked && u.applied);
    }

    #[test]
    fn smith_predictor_subtracts_last_pd() {
        let mut p = PhaseLockLoop::new();
        let u1 = p.update(800, true); // p_corr = 100, last_pd = 100
        assert_eq!(u1.p_corr_ns, 100);
        let u2 = p.update(800, true); // pred = 800 - 100 = 700
        assert_eq!(u2.predicted_phase_ns, 700);
        assert_eq!(u2.p_corr_ns, 700 / 8);
    }

    #[test]
    fn locks_after_lock_hold_in_window_edges() {
        let mut p = PhaseLockLoop::new();
        // 4 edges within the lock window: not yet locked (lock attained on the 5th).
        for _ in 0..4 {
            assert!(!p.update(100, true).locked);
        }
        // The control on this 5th edge still sees the pre-edge state (not yet locked)...
        assert!(!p.update(100, true).locked);
        // ...but now lock_cnt has reached lock_hold.
        assert!(p.is_locked());
    }

    #[test]
    fn i_term_integrates_only_once_locked() {
        let mut p = PhaseLockLoop::new();
        run(&mut p, 100, 5); // reach lock
        assert!(p.is_locked());
        assert_eq!(p.freq_trim_mppb(), 0); // nothing integrated before lock
        let u = p.update(100, true); // locked now → integrate
        assert!(u.locked);
        assert!(u.freq_trim_mppb < 0);
        assert_eq!(u.freq_trim_mppb, p.freq_trim_mppb());
    }

    #[test]
    fn invalid_holds_trim_and_lock() {
        let mut p = PhaseLockLoop::new();
        run(&mut p, 100, 6); // locked + some trim
        let trim = p.freq_trim_mppb();
        assert!(p.is_locked());
        let u = p.update(999_999, false); // invalid: no correction, state held
        assert!(!u.applied);
        assert_eq!(u.p_corr_ns, 0);
        assert_eq!(u.d_corr_ns, 0);
        assert_eq!(u.freq_trim_mppb, trim); // trim held
        assert!(p.is_locked()); // lock held
    }

    #[test]
    fn locked_outlier_is_rejected_then_accepted_after_max() {
        let mut p = PhaseLockLoop::new();
        run(&mut p, 100, 6); // locked
        // A big spike while locked is rejected (held), up to outlier_max times.
        for _ in 0..p.config().outlier_max {
            let u = p.update(50_000, true);
            assert!(u.rejected_outlier && !u.applied);
            assert_eq!(u.phase_corr_ns, 0);
        }
        // The next one is accepted as a real disturbance.
        let u = p.update(50_000, true);
        assert!(!u.rejected_outlier && u.applied);
    }

    #[test]
    fn p_only_mode_has_no_integral_or_derivative() {
        let mut p = PhaseLockLoop::with_config(PhaseLockLoopConfig {
            mode: LoopMode::P,
            ..PhaseLockLoopConfig::DEFAULT
        });
        let u = run(&mut p, 100, 8);
        assert_eq!(u.freq_trim_mppb, 0); // I disabled → trim forced to 0
        assert_eq!(u.d_corr_ns, 0); // D disabled
        assert_eq!(u.p_corr_ns, u.predicted_phase_ns / 16); // no Smith → kp_inv 16
    }

    #[test]
    fn trim_is_clamped() {
        let mut p = PhaseLockLoop::new();
        run(&mut p, 100, 5); // lock
        // A large persistent phase would integrate without bound; ensure it clamps.
        for _ in 0..10_000 {
            p.update(100_000, true);
        }
        assert!(p.freq_trim_mppb().abs() <= p.config().trim_max_mppb);
    }
}
