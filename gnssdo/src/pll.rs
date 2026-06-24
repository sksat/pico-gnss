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
    /// Integral-enable window (ns): the I term integrates while the phase is within this band, even
    /// *before* full lock. It must be wider than `lock_ns`: P alone settles at a non-zero phase, so
    /// if I only ran once locked (within `lock_ns`) but P can't get inside `lock_ns`, lock — and
    /// hence I — would never engage. The wider band lets the integrator pull the offset into lock.
    /// Default 5 µs.
    pub i_enable_ns: i64,
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
    /// Adaptive-gain "calm" band (ns). While `|predicted_phase|` is within this, the integral runs at
    /// the aggressive `i_den` (tight tracking of slow/thermal drift); beyond it the disturbance is
    /// large (a thermal shock), so the integral is slowed by `i_den_disturbed_shift` to avoid windup
    /// and overshoot. Gentle warming keeps the phase small (the loop tracks it), so it stays in the
    /// aggressive regime; a violent shock blows the phase up and drops to the conservative regime.
    /// Default = `lock_ns`.
    pub calm_ns: i64,
    /// Integral slow-down outside the calm band: effective `i_den` becomes `i_den << shift`. 0 disables
    /// adaptation (the integral always uses `i_den`), preserving the fixed-gain behaviour. Default 0.
    pub i_den_disturbed_shift: u32,
}

impl PhaseLockLoopConfig {
    /// The measured σ≈35 ns production tuning. `const` so [`PhaseLockLoop::new`] is `const`.
    pub const DEFAULT: Self = Self {
        mode: LoopMode::PidSmith,
        deadband_ns: 0,
        lock_ns: 1_000,
        lock_hold: 5,
        i_enable_ns: 5_000,
        outlier_ns: 3_000,
        outlier_max: 12,
        i_den: 128,
        trim_max_mppb: 3_000_000,
        d_den: 4,
        calm_ns: 1_000,
        i_den_disturbed_shift: 0,
    };

    /// The **naive PID** preset (P + I + D, *no* Smith predictor) — the textbook type-II loop from
    /// the early report, kept selectable so the Smith predictor's contribution can be measured. The
    /// missing predictor forces the gentler `kp_inv=16` (see [`LoopMode::kp_inv`]).
    pub const NAIVE_PID: Self = Self {
        mode: LoopMode::Pid,
        ..Self::DEFAULT
    };

    /// Naive **PI** preset (no derivative, no Smith) — type-II without the D term.
    pub const NAIVE_PI: Self = Self {
        mode: LoopMode::Pi,
        ..Self::DEFAULT
    };

    /// **Proportional-only** preset — the simplest servo (type-I, settles at a non-zero offset).
    pub const P_ONLY: Self = Self {
        mode: LoopMode::P,
        ..Self::DEFAULT
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

    /// Set the integral denominator at runtime, preserving the integrated trim and lock state.
    /// Larger `i_den` = slower integration = lower loop bandwidth = longer natural period
    /// (≈ 2π√`i_den` edges). Used by the firmware's experiment harness to sweep `i_den` under
    /// matched conditions (the closed-loop output-phase wander is the underdamped type-II mode at
    /// that natural period, so sweeping `i_den` moves the mode without re-locking the loop).
    pub fn set_i_den(&mut self, i_den: i64) {
        self.config.i_den = i_den;
    }

    /// Set the derivative denominator at runtime (experiment harness: sweep damping under matched
    /// conditions). Larger `d_den` = weaker derivative. Critical damping of this Smith-delayed
    /// integer loop needs a *joint* move (i_den up + d_den up together), so the harness sets both.
    pub fn set_d_den(&mut self, d_den: i64) {
        self.config.d_den = d_den;
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
                // I term: integrate the predicted phase into the frequency trim (milli-ppb) while the
                // phase is within the I-enable band — wider than the lock window, so the integrator can
                // pull a steady-state offset that P alone leaves *outside* `lock_ns` into lock (and
                // thus drive it to zero). Reset when the I term is off.
                if self.config.mode.use_i() {
                    if ctrl.abs() < self.config.i_enable_ns {
                        // Adaptive integral gain: aggressive (i_den) while calm, slowed when the phase
                        // is disturbed (a shock), to track slow thermal drift tightly without winding
                        // up/overshooting on a fast disturbance.
                        let i_den_eff = if pred.abs() < self.config.calm_ns {
                            self.config.i_den
                        } else {
                            self.config.i_den << self.config.i_den_disturbed_shift.min(20)
                        };
                        // `.max(1)`: `i_den` is settable via `set_i_den`, and integer divide-by-zero
                        // panics on the no_std firmware — guard it structurally.
                        self.trim_mppb = (self.trim_mppb - pred * 1000 / i_den_eff.max(1))
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
                // D term (only while locked). `.max(1)` guards `set_d_den(0)` (no_std divide-by-zero).
                if self.config.mode.use_d() && locked {
                    d_corr = ((pred - self.last_pred) / self.config.d_den.max(1))
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

        // Only carry the Smith/derivative history forward when a correction was actually applied. On
        // a rejected outlier or an invalid (held) edge, keeping the last *applied* values avoids a
        // spurious D kick of `(normal − spike) / d_den` on the next good edge.
        if applied {
            self.last_pred = pred;
            self.last_pd = p_corr + d_corr;
        }
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

/// `PhaseLockLoop` as a pluggable [`PhaseController`](crate::PhaseController): `step` is a lossless
/// view of the inherent [`update`](PhaseLockLoop::update) (guarded bit-for-bit by a test), so the
/// firmware/tests keep the richer `update` while the comparison harness drives it through the trait.
impl crate::PhaseController for PhaseLockLoop {
    fn step(&mut self, input: crate::ControlInput) -> crate::ControlOutput {
        let u = self.update(input.err_ns, input.valid);
        crate::ControlOutput {
            trim_mppb: u.freq_trim_mppb,
            pcorr_ns: u.phase_corr_ns,
            applied: u.applied,
            locked: u.locked,
            rejected: u.rejected_outlier,
            dbg: crate::ControlDebug {
                pred_ns: u.predicted_phase_ns,
                p_ns: u.p_corr_ns,
                d_ns: u.d_corr_ns,
                // This loop's "residual frequency" estimate *is* its integrated trim.
                observer_freq_mppb: u.freq_trim_mppb,
                state: 0,
            },
        }
    }

    fn is_locked(&self) -> bool {
        PhaseLockLoop::is_locked(self)
    }

    fn start_segment(&mut self, init: crate::ControlInit) {
        // Fair cross-controller switch: blank every per-edge history (Smith/derivative + lock +
        // outlier counters) and seed the residual trim so the output frequency stays continuous.
        // Lock is blanked → the segment begins unlocked (warmup, excluded from comparison); the
        // first post-switch correction is P-only (no spurious D kick off stale history).
        self.lock_cnt = 0;
        self.reject_cnt = 0;
        self.last_pred = 0;
        self.last_pd = 0;
        self.trim_mppb = init.residual_trim_mppb;
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
        assert_eq!(p.config().i_enable_ns, 5_000);
        assert_eq!(p.config().i_den, 128);
    }

    #[test]
    fn p_term_first_edge_uses_kp_inv_8_and_no_smith_history() {
        let mut p = PhaseLockLoop::new();
        // First edge beyond the I-enable band (8 µs > 5 µs): last_pd=0 so pred=ctrl; PidSmith
        // kp_inv=8; not locked so no D; phase outside the band so no I yet.
        let u = p.update(8_000, true);
        assert_eq!(u.predicted_phase_ns, 8_000);
        assert_eq!(u.p_corr_ns, 8_000 / 8);
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
    fn i_term_integrates_within_enable_band_before_lock() {
        // A phase inside the I-enable band but outside the lock window (1 µs < 2 µs < 5 µs):
        // not locked, yet the integrator engages and pulls toward zero. This is the fix for
        // "P alone parks outside lock_ns so lock (hence I) never engages".
        let mut p = PhaseLockLoop::new();
        let u = p.update(2_000, true);
        assert!(!u.locked); // 2 µs > lock_ns, so not locked...
        assert!(u.freq_trim_mppb < 0); // ...but I still integrated (within i_enable_ns)
        // A phase beyond the band does NOT integrate.
        let mut q = PhaseLockLoop::new();
        assert_eq!(q.update(8_000, true).freq_trim_mppb, 0);
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
    fn outlier_reject_does_not_poison_smith_history() {
        let mut p = PhaseLockLoop::new();
        run(&mut p, 100, 6); // locked, history settled near phase 100
        // A rejected spike must not update last_pred/last_pd, or the next good edge's D term would
        // see (normal − spike) / d_den and kick hard.
        let u_spike = p.update(50_000, true);
        assert!(u_spike.rejected_outlier && !u_spike.applied);
        let u_next = p.update(100, true);
        assert!(u_next.applied);
        assert!(u_next.d_corr_ns.abs() < 1_000); // no (100 − 50_000)/4 ≈ −12_475 kick
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
    fn adaptive_gain_slows_integral_when_disturbed() {
        // 適応積分: |pred| が calm_ns 内なら i_den (aggressive)、超えたら i_den<<shift (slow)。
        // Smith/D を外した Pi で pred=ctrl にし、1 エッジの trim 増分で実効 i_den を確認。
        let cfg = |shift| PhaseLockLoopConfig {
            mode: LoopMode::Pi,
            i_den: 32,
            calm_ns: 1_000,
            i_den_disturbed_shift: shift,
            ..PhaseLockLoopConfig::DEFAULT
        };
        // calm (500 < 1000): shift によらず i_den=32。
        let mut p = PhaseLockLoop::with_config(cfg(2));
        assert_eq!(p.update(500, true).freq_trim_mppb, -500 * 1000 / 32);
        // disturbed (2000 > 1000) + shift=2: i_den を <<2 = 128 に鈍化。
        let mut q = PhaseLockLoop::with_config(cfg(2));
        assert_eq!(q.update(2000, true).freq_trim_mppb, -2000 * 1000 / 128);
        // 同じ disturbed phase でも shift=0 なら適応無効で i_den=32 のまま。
        let mut r = PhaseLockLoop::with_config(cfg(0));
        assert_eq!(r.update(2000, true).freq_trim_mppb, -2000 * 1000 / 32);
    }

    #[test]
    fn trim_is_clamped() {
        let mut p = PhaseLockLoop::new();
        run(&mut p, 100, 5); // lock
        // A persistent in-band phase would integrate without bound; ensure it clamps. (Use a phase
        // inside i_enable_ns so the integrator actually runs; >band would be gated out.)
        for _ in 0..10_000 {
            p.update(4_000, true);
        }
        assert!(p.freq_trim_mppb().abs() <= p.config().trim_max_mppb);
    }

    #[test]
    fn set_i_den_changes_integration_rate_live() {
        // set_i_den retunes the integral denominator in place (firmware experiment harness sweeps it).
        // Use a Pi loop (pred=ctrl, no Smith/D) so the trim increment is exactly -pred*1000/i_den.
        let cfg = PhaseLockLoopConfig {
            mode: LoopMode::Pi,
            ..PhaseLockLoopConfig::DEFAULT
        };
        let mut p = PhaseLockLoop::with_config(cfg);
        assert_eq!(p.config().i_den, 128);
        assert_eq!(p.update(2_000, true).freq_trim_mppb, -2_000 * 1000 / 128);
        // Loosen: the next in-band edge integrates at the new (slower) rate, trim preserved.
        p.set_i_den(512);
        assert_eq!(p.config().i_den, 512);
        let before = p.freq_trim_mppb();
        let u = p.update(2_000, true);
        assert_eq!(u.freq_trim_mppb, before - 2_000 * 1000 / 512);
    }

    #[test]
    fn zero_denominator_setters_do_not_panic() {
        // `set_i_den(0)` / `set_d_den(0)` must not cause an integer divide-by-zero panic (the no_std
        // firmware has no panic budget). Drive an in-band, locked edge so both the I and D paths run.
        let mut p = PhaseLockLoop::new();
        run(&mut p, 100, 8); // lock
        p.set_i_den(0);
        p.set_d_den(0);
        let _ = p.update(2_000, true); // in i_enable band + locked → exercises both divisors
    }

    #[test]
    fn trait_step_matches_inherent_update_bit_for_bit() {
        use crate::{ControlInput, PhaseController};
        // The trait `step` must be a lossless view of the inherent `update` — the firmware and the
        // host comparison harness have to agree edge-for-edge, or a "method difference" could really
        // be a trait-vs-inherent divergence. Drive an identical (err,valid) sequence through both and
        // require the actuator outputs and telemetry match exactly. The sequence exercises reacq (big
        // offset), lock, an outlier spike, and an invalid (holdover) edge.
        let mut a = PhaseLockLoop::new();
        let mut b = PhaseLockLoop::new();
        let seq: &[(i64, bool)] = &[
            (50_000, true),
            (40_000, true),
            (3_000, true),
            (800, true),
            (100, true),
            (100, true),
            (100, true),
            (100, true),
            (50_000, true),
            (120, true),
            (999_999, false),
            (90, true),
            (-300, true),
            (2_000, true),
            (0, true),
        ];
        for &(err, valid) in seq {
            let u = a.update(err, valid);
            let o = PhaseController::step(&mut b, ControlInput { err_ns: err, valid });
            assert_eq!(o.trim_mppb, u.freq_trim_mppb);
            assert_eq!(o.pcorr_ns, u.phase_corr_ns);
            assert_eq!(o.applied, u.applied);
            assert_eq!(o.locked, u.locked);
            assert_eq!(o.rejected, u.rejected_outlier);
            assert_eq!(o.dbg.pred_ns, u.predicted_phase_ns);
            assert_eq!(o.dbg.p_ns, u.p_corr_ns);
            assert_eq!(o.dbg.d_ns, u.d_corr_ns);
        }
        assert_eq!(PhaseController::is_locked(&b), a.is_locked());
    }

    #[test]
    fn start_segment_blanks_history_and_seeds_residual_trim() {
        use crate::{ControlInit, ControlInput, PhaseController};
        let mut p = PhaseLockLoop::new();
        // Lock and accumulate trim + Smith/D history.
        run(&mut p, 100, 8);
        assert!(p.is_locked());
        // Switch into a fresh comparison segment with a seeded residual trim (output freq continuous).
        PhaseController::start_segment(&mut p, ControlInit { residual_trim_mppb: 12_345 });
        assert!(!PhaseController::is_locked(&p)); // lock blanked → warmup
        assert_eq!(p.freq_trim_mppb(), 12_345); // residual trim seeded
        // First post-switch edge: D needs lock (just blanked), so the correction is P-only — no kick
        // off stale Smith/derivative history.
        let o = PhaseController::step(&mut p, ControlInput { err_ns: 800, valid: true });
        assert_eq!(o.dbg.d_ns, 0);
        assert_eq!(o.dbg.p_ns, 800 / 8);
    }

    #[test]
    fn stronger_d_destabilizes_the_underdamped_mode_not_the_reverse() {
        // The disciplined output's slow phase wander is this loop's underdamped type-II mode. The
        // continuous-time intuition "add D (or raise P) to damp it" is INVERTED here: with the Smith
        // one-edge delay and integer truncation, a closed-loop step response rings HARDER with
        // stronger D. This was confirmed on hardware (the wander is reception-limited; tightening the
        // loop does not help and can destabilize). Guard it so a future "damping fix" can't silently
        // re-introduce the instability. (kp_inv is fixed at 8 for Smith; d_den is the available knob.)
        fn step_ringdown(d_den: i64) -> (i64, usize) {
            let mut pll = PhaseLockLoop::with_config(PhaseLockLoopConfig {
                d_den,
                ..PhaseLockLoopConfig::DEFAULT // production: PidSmith, i_den=128, d_den=4
            });
            for _ in 0..40 {
                pll.update(0, true); // settle + lock at zero phase
            }
            let mut p: i64 = 2_000; // apply a 2 µs phase step
            let (mut peak, mut prev, mut zc) = (0i64, p, 0usize);
            for _ in 0..500 {
                let u = pll.update(p, true);
                p += u.freq_trim_mppb / 1000 - u.phase_corr_ns; // closed-loop plant
                peak = peak.max(p.abs());
                if (prev > 0) != (p > 0) {
                    zc += 1; // count zero-crossings (oscillation)
                }
                prev = p;
            }
            (peak, zc)
        }
        let (peak_prod, _zc_prod) = step_ringdown(4); // production
        let (peak_mid, _) = step_ringdown(2); // stiffer D
        let (peak_stiff, _) = step_ringdown(1); // much stiffer D
        // Production settles to a bounded ring-down (does not blow past a small multiple of the step).
        assert!(
            peak_prod <= 4_000,
            "production (d_den=4) step peak {peak_prod}ns should stay bounded"
        );
        // Stiffening D rings monotonically HARDER, not softer — the inversion. A future dev must not
        // "fix" the wander by lowering d_den.
        assert!(
            peak_stiff > peak_mid && peak_mid > peak_prod,
            "stiffer D should ring harder (inverted): d1={peak_stiff} d2={peak_mid} d4={peak_prod}"
        );
    }
}
