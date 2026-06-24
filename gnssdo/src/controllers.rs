//! Alternative [`PhaseController`](crate::PhaseController) strategies, so the historical and
//! candidate output-phase methods can all be selected and compared under matched conditions (see
//! [`control`](crate::control) for why a common abstraction matters — the measurement trap).
//!
//! - [`OpenLoopFf`]: the **loopback-less** configuration from `report/REPORT.md` ("ループバック無しの
//!   構成"). With no hardware loopback there is no honest output-vs-reference phase to close, so the
//!   phase servo is dropped and the output rides the crystal feedforward open-loop. Represented here
//!   as a controller that emits no phase correction and only holds a residual trim.
//! - [`AlphaBetaBoost`]: the integer realization of the "clean state estimate + transient boost"
//!   idea (the implementable reduction of the Kalman+nonlinear-gain experiment). A fixed-gain α-β
//!   phase observer feeds a low, fixed correction gain in steady state; a **hysteresis state machine**
//!   raises the observer/correction gains only on a genuine transient (large innovation, accepted
//!   disturbance, or lost lock) — never on steady wander, which is what re-excites the loop
//!   resonance. This is the candidate that aims to recover fast *and* stay quiet without a mode flag
//!   the steady wander keeps crossing.
//! - [`IntegralRework`]: the flag-free continuous integrator (no I-enable gate; integral *input*
//!   clamped + back-calculation anti-windup) — the host experiment showed it is a clean no-op vs the
//!   gated integrator on the model, kept here so that result is reproducible in the firmware library.
//!
//! The naive (non-Smith) P/PD/PI/PID family is not here: it is the existing
//! [`PhaseLockLoop`](crate::PhaseLockLoop) under a non-`PidSmith` [`LoopMode`](crate::LoopMode), with
//! presets on [`PhaseLockLoopConfig`](crate::PhaseLockLoopConfig).

use crate::{
    ControlDebug, ControlInit, ControlInput, ControlOutput, PhaseController, PhaseLockLoop,
};

/// Safety clamp on one edge's phase correction (ns) — a larger move is a glitch, not control.
/// Matches `pll`'s clamp so every controller has the same actuator limit.
const CORR_CLAMP_NS: i64 = 100_000_000;

// ---------------------------------------------------------------------------------------------
// Controller — a runtime-selectable enum over the strategies (no_std, no-alloc dispatch).
// ---------------------------------------------------------------------------------------------

/// A runtime-selectable phase controller for **same-boot cycling**: `no_std`/no-alloc enum dispatch
/// over the library's strategies (vs `Box<dyn>`, which needs an allocator the firmware lacks).
///
/// This is the vehicle for the only reception-independent comparison available under the measurement
/// trap (see [`control`](crate::control)): the firmware holds a list of `Controller`s, cycles them in
/// one boot, and injects PRBS into each, so method differences are measured against the *same*
/// reception. Switch fairly with [`start_segment`](PhaseController::start_segment) — seed the residual
/// trim so the output frequency is continuous and blank every per-edge history. The
/// [`PhaseLockLoop`] variant also covers the naive-PID family (via
/// [`PhaseLockLoopConfig`](crate::PhaseLockLoopConfig) presets) and the production Smith loop.
#[derive(Debug)]
pub enum Controller {
    /// Type-II PID, with or without the Smith predictor (production and the naive-PID presets).
    Pll(PhaseLockLoop),
    /// Flag-free continuous integrator.
    IntegRework(IntegralRework),
    /// Fixed-gain α-β observer + residual-triggered transient boost.
    AbBoost(AlphaBetaBoost),
    /// Open-loop feedforward (the loopback-less configuration).
    OpenLoop(OpenLoopFf),
}

impl Controller {
    /// A short stable name for logging / the comparison harness.
    pub fn name(&self) -> &'static str {
        match self {
            Controller::Pll(_) => "pll",
            Controller::IntegRework(_) => "integ_rework",
            Controller::AbBoost(_) => "ab_boost",
            Controller::OpenLoop(_) => "open_loop",
        }
    }
}

impl PhaseController for Controller {
    fn step(&mut self, input: ControlInput) -> ControlOutput {
        match self {
            Controller::Pll(c) => c.step(input),
            Controller::IntegRework(c) => c.step(input),
            Controller::AbBoost(c) => c.step(input),
            Controller::OpenLoop(c) => c.step(input),
        }
    }

    fn is_locked(&self) -> bool {
        match self {
            Controller::Pll(c) => PhaseController::is_locked(c),
            Controller::IntegRework(c) => c.is_locked(),
            Controller::AbBoost(c) => c.is_locked(),
            Controller::OpenLoop(c) => c.is_locked(),
        }
    }

    fn start_segment(&mut self, init: ControlInit) {
        match self {
            Controller::Pll(c) => c.start_segment(init),
            Controller::IntegRework(c) => c.start_segment(init),
            Controller::AbBoost(c) => c.start_segment(init),
            Controller::OpenLoop(c) => c.start_segment(init),
        }
    }
}

// ---------------------------------------------------------------------------------------------
// OpenLoopFf — the loopback-less / open-loop configuration.
// ---------------------------------------------------------------------------------------------

/// Open-loop feedforward: **no phase servo**. Models the loopback-less build (`report/REPORT.md`'s
/// "ループバック無しの構成"), where without the hardware loopback there is no honest output phase to
/// close, so the output rides the crystal feedforward and any fixed offset is burned, not measured.
///
/// As a [`PhaseController`] it emits `pcorr_ns = 0` always and holds whatever residual trim it was
/// seeded with via [`start_segment`](PhaseController::start_segment) (0 by default). It never claims
/// lock (there is no phase feedback to lock to); `err_ns` is recorded for telemetry only.
#[derive(Debug, Default)]
pub struct OpenLoopFf {
    residual_trim_mppb: i64,
}

impl OpenLoopFf {
    /// Create an open-loop controller (rides the external feedforward only).
    pub const fn new() -> Self {
        Self {
            residual_trim_mppb: 0,
        }
    }
}

impl PhaseController for OpenLoopFf {
    fn step(&mut self, input: ControlInput) -> ControlOutput {
        ControlOutput {
            trim_mppb: self.residual_trim_mppb,
            pcorr_ns: 0,
            applied: input.valid,
            locked: false,
            rejected: false,
            dbg: ControlDebug {
                pred_ns: input.err_ns,
                observer_freq_mppb: self.residual_trim_mppb,
                ..ControlDebug::default()
            },
        }
    }

    fn is_locked(&self) -> bool {
        false
    }

    fn start_segment(&mut self, init: ControlInit) {
        self.residual_trim_mppb = init.residual_trim_mppb;
    }
}

// ---------------------------------------------------------------------------------------------
// AlphaBetaBoost — fixed-gain α-β observer + residual-triggered transient boost.
// ---------------------------------------------------------------------------------------------

/// Tuning for [`AlphaBetaBoost`]. Gains are inverse (a gain of `1/x` is stored as `x`), keeping the
/// math integer and the "larger = gentler" reading. Defaults lock and stay bounded on the host
/// plant model; the harness sweeps them (do not over-fit the model — hardware PRBS decides).
#[derive(Debug, Clone, Copy)]
pub struct AlphaBetaBoostConfig {
    /// Steady α-β observer gains (inverse): `phase += r/steady_alpha_inv`, `freq += r·1000/steady_beta_inv`.
    pub steady_alpha_inv: i64,
    pub steady_beta_inv: i64,
    /// Steady phase-correction gain (inverse): `pcorr = phase_hat / steady_pgain_inv`. Low on purpose
    /// — a gentle steady gain is what keeps the underdamped loop from ringing without a Smith term.
    pub steady_pgain_inv: i64,
    /// Boost α-β observer gains (inverse) — believe a big mismatch fast (this is the role the Kalman's
    /// adaptive covariance played; removing it cost most of the reacquisition speed).
    pub fast_alpha_inv: i64,
    pub fast_beta_inv: i64,
    /// Boost phase-correction gain (inverse) — stronger, to pull a real disturbance in quickly.
    pub fast_pgain_inv: i64,
    /// Innovation magnitude (ns) that *enters* boost. Keyed to the **innovation** (surprise vs the
    /// observer's prediction), not `|phase|`, so steady wander does not trigger it.
    pub innov_enter_ns: i64,
    /// Max edges to hold boost before forcing back to steady (a backstop, not the normal exit).
    pub boost_max_edges: u32,
    /// Exit boost once `|err|` stays within this (ns) for `exit_hold` edges.
    pub exit_lock_ns: i64,
    pub exit_hold: u32,
    /// Lock window (ns) and consecutive in-window edges to declare lock.
    pub lock_ns: i64,
    pub lock_hold: u32,
    /// Post-lock single-sample outlier threshold (ns) and how many to hold before accepting one as a
    /// real disturbance (which then enters boost).
    pub outlier_ns: i64,
    pub outlier_max: u32,
    /// Clamp on the integrated frequency trim (milli-ppb).
    pub trim_max_mppb: i64,
}

impl AlphaBetaBoostConfig {
    /// Default tuning (locks and stays bounded on the model; harness-tunable).
    pub const DEFAULT: Self = Self {
        steady_alpha_inv: 4,
        steady_beta_inv: 64,
        steady_pgain_inv: 8,
        fast_alpha_inv: 2,
        fast_beta_inv: 8,
        fast_pgain_inv: 3,
        innov_enter_ns: 3_000,
        boost_max_edges: 40,
        exit_lock_ns: 1_000,
        exit_hold: 5,
        lock_ns: 1_000,
        lock_hold: 5,
        outlier_ns: 3_000,
        outlier_max: 12,
        trim_max_mppb: 3_000_000,
    };
}

impl Default for AlphaBetaBoostConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Fixed-gain α-β output-phase observer with a residual-triggered transient boost. See the
/// [module docs](self). Integer-only / `no_std`. Telemetry `state`: 0 = steady, 1 = boosting.
#[derive(Debug)]
pub struct AlphaBetaBoost {
    cfg: AlphaBetaBoostConfig,
    phase_hat: i64,      // estimated output phase error (ns)
    drift_hat_mppb: i64, // estimated *uncorrected* drift rate (milli-ppb); the trim cancels it
    last_trim_mppb: i64, // trim applied last edge (for the closed-loop predict)
    last_pcorr_ns: i64,  // phase correction applied last edge (Smith-like predict term)
    boosting: bool,
    boost_edges: u32,
    exit_cnt: u32,
    lock_cnt: u32,
    reject_cnt: u32,
}

impl AlphaBetaBoost {
    /// Create with the default tuning.
    pub const fn new() -> Self {
        Self::with_config(AlphaBetaBoostConfig::DEFAULT)
    }

    /// Create with a given tuning (`const` so it can initialize a `static`).
    pub const fn with_config(cfg: AlphaBetaBoostConfig) -> Self {
        Self {
            cfg,
            phase_hat: 0,
            drift_hat_mppb: 0,
            last_trim_mppb: 0,
            last_pcorr_ns: 0,
            boosting: false,
            boost_edges: 0,
            exit_cnt: 0,
            lock_cnt: 0,
            reject_cnt: 0,
        }
    }

    /// Current configuration.
    pub fn config(&self) -> &AlphaBetaBoostConfig {
        &self.cfg
    }

    /// Current residual frequency trim (milli-ppb) = −(drift estimate), clamped. Mirrors
    /// [`PhaseLockLoop::freq_trim_mppb`](crate::PhaseLockLoop::freq_trim_mppb) for logging/tests.
    pub fn freq_trim_mppb(&self) -> i64 {
        (-self.drift_hat_mppb).clamp(-self.cfg.trim_max_mppb, self.cfg.trim_max_mppb)
    }

    /// Force the boost backstop forward and drop out of boost once it expires — runs on *every* edge
    /// (applied, rejected, or holdover) so a periodic outlier cannot starve the timer and latch the
    /// boost on. Returns the (possibly updated) boost state.
    fn tick_boost_backstop(&mut self) {
        if self.boosting {
            self.boost_edges = self.boost_edges.saturating_add(1);
            if self.boost_edges >= self.cfg.boost_max_edges {
                self.boosting = false;
            }
        }
    }
}

impl Default for AlphaBetaBoost {
    fn default() -> Self {
        Self::new()
    }
}

impl PhaseController for AlphaBetaBoost {
    fn step(&mut self, input: ControlInput) -> ControlOutput {
        let z = input.err_ns;
        let locked = self.lock_cnt >= self.cfg.lock_hold;
        let held_trim =
            (-self.drift_hat_mppb).clamp(-self.cfg.trim_max_mppb, self.cfg.trim_max_mppb);

        if !input.valid {
            // Holdover: hold trim + lock, emit no correction, observer frozen. The backstop still
            // ticks so a long holdover cannot leave the loop latched in boost.
            self.tick_boost_backstop();
            return ControlOutput {
                trim_mppb: held_trim,
                pcorr_ns: 0,
                applied: false,
                locked,
                rejected: false,
                dbg: ControlDebug {
                    pred_ns: self.phase_hat,
                    observer_freq_mppb: self.drift_hat_mppb,
                    state: self.boosting as u8,
                    ..ControlDebug::default()
                },
            };
        }

        // Closed-loop α-β prediction. The trim applied last edge cancels the estimated drift, so the
        // only motion the model expects since the last edge is the drift residual minus the phase
        // correction we applied (Smith-like). Writing it with the applied last_trim makes it exact and
        // — crucially — phantom-free across a fair segment switch (drift_hat = −seed, last_trim = seed
        // ⇒ phase_pred = 0 at a steady edge, so the first emitted trim equals the seed; see
        // start_segment). Ignoring the applied control (the old `phase_hat + drift/1000`) both broke
        // switch continuity and lagged on sustained drift.
        let phase_pred = self.phase_hat + (self.drift_hat_mppb + self.last_trim_mppb) / 1000
            - self.last_pcorr_ns;
        let innov = z - phase_pred;

        // Isolated post-lock spike: reject (hold). Backstop ticks regardless (anti-latch).
        if locked && z.abs() > self.cfg.outlier_ns && self.reject_cnt < self.cfg.outlier_max {
            self.reject_cnt += 1;
            self.tick_boost_backstop();
            return ControlOutput {
                trim_mppb: held_trim,
                pcorr_ns: 0,
                applied: false,
                locked,
                rejected: true,
                dbg: ControlDebug {
                    pred_ns: phase_pred,
                    observer_freq_mppb: self.drift_hat_mppb,
                    state: self.boosting as u8,
                    ..ControlDebug::default()
                },
            };
        }
        // A disturbance accepted after a full reject run — but only if THIS edge is itself out of the
        // lock window. A zero-error edge that merely broke the reject streak is not a disturbance and
        // must not trigger boost (the phantom-boost-latch defect).
        let accepted_disturbance =
            self.reject_cnt >= self.cfg.outlier_max && z.abs() >= self.cfg.lock_ns;
        self.reject_cnt = 0;

        // Boost hysteresis. Enter ONLY on a genuine surprise — a large *innovation* (a real step or a
        // lost lock both surface as a big innovation) or an accepted persistent disturbance. Keyed to
        // the innovation, never to raw |z|, so steady wander crossing the lock window cannot flap the
        // aggressive regime (which would re-excite the very resonance this controller exists to avoid).
        if !self.boosting {
            if innov.abs() > self.cfg.innov_enter_ns || accepted_disturbance {
                self.boosting = true;
                self.boost_edges = 0;
                self.exit_cnt = 0;
            }
        } else {
            self.boost_edges = self.boost_edges.saturating_add(1);
            if z.abs() < self.cfg.exit_lock_ns {
                self.exit_cnt += 1;
            } else {
                self.exit_cnt = 0;
            }
            if self.exit_cnt >= self.cfg.exit_hold || self.boost_edges >= self.cfg.boost_max_edges {
                self.boosting = false;
            }
        }

        // Gains by regime. `.max(1)` keeps every divisor ≥ 1: an integer divide-by-zero would panic
        // on the no_std firmware (and the gain fields are a public, settable API), and a non-positive
        // gain would invert the feedback sign — so this both guards the panic and pins the precondition.
        let (alpha_inv, beta_inv, pgain_inv) = if self.boosting {
            (
                self.cfg.fast_alpha_inv.max(1),
                self.cfg.fast_beta_inv.max(1),
                self.cfg.fast_pgain_inv.max(1),
            )
        } else {
            (
                self.cfg.steady_alpha_inv.max(1),
                self.cfg.steady_beta_inv.max(1),
                self.cfg.steady_pgain_inv.max(1),
            )
        };

        // α-β update: phase correction + drift integrator.
        self.phase_hat = phase_pred + innov / alpha_inv;
        self.drift_hat_mppb = (self.drift_hat_mppb + innov * 1000 / beta_inv)
            .clamp(-self.cfg.trim_max_mppb, self.cfg.trim_max_mppb);

        // Actuators: cancel the drift (trim) and pull the estimated phase to zero (pcorr).
        let trim_mppb =
            (-self.drift_hat_mppb).clamp(-self.cfg.trim_max_mppb, self.cfg.trim_max_mppb);
        let pcorr_ns = (self.phase_hat / pgain_inv).clamp(-CORR_CLAMP_NS, CORR_CLAMP_NS);

        // Lock tracking on the raw measurement.
        self.lock_cnt = if z.abs() < self.cfg.lock_ns {
            (self.lock_cnt + 1).min(self.cfg.lock_hold)
        } else {
            0
        };

        self.last_trim_mppb = trim_mppb;
        self.last_pcorr_ns = pcorr_ns;

        ControlOutput {
            trim_mppb,
            pcorr_ns,
            applied: true,
            locked,
            rejected: false,
            dbg: ControlDebug {
                pred_ns: phase_pred,
                p_ns: pcorr_ns,
                d_ns: 0,
                observer_freq_mppb: self.drift_hat_mppb,
                state: self.boosting as u8,
            },
        }
    }

    fn is_locked(&self) -> bool {
        self.lock_cnt >= self.cfg.lock_hold
    }

    fn start_segment(&mut self, init: ControlInit) {
        // Fair segment start: seed the drift estimate so the first emitted trim equals the seed AND
        // the first predict at a steady (err==0) edge yields zero innovation — output frequency
        // continuous, pcorr 0, no phantom kick. Blank the phase estimate and every counter/state.
        self.drift_hat_mppb = -init.residual_trim_mppb;
        self.last_trim_mppb = init.residual_trim_mppb;
        self.phase_hat = 0;
        self.last_pcorr_ns = 0;
        self.boosting = false;
        self.boost_edges = 0;
        self.exit_cnt = 0;
        self.lock_cnt = 0;
        self.reject_cnt = 0;
    }
}

// ---------------------------------------------------------------------------------------------
// IntegralRework — flag-free continuous integrator (no I-enable gate).
// ---------------------------------------------------------------------------------------------

/// Tuning for [`IntegralRework`]. Mirrors the production loop's P/D/Smith path but reworks the
/// integral to be **continuous** (no I-enable gate): always integrate, but clamp the integral
/// *input* to `±e_i_ns` so a big transient cannot wind it up, and bleed back any actuator-trim
/// saturation with `kbc_inv` (back-calculation). Same P/D as the Smith production loop.
#[derive(Debug, Clone, Copy)]
pub struct IntegralReworkConfig {
    /// Inverse proportional gain (Smith-compensated → 8, like the production loop).
    pub kp_inv: i64,
    /// Derivative denominator.
    pub d_den: i64,
    /// Integral denominator (per-edge trim step = −clamp(pred)·1000 / i_den).
    pub i_den: i64,
    /// Integral-input clamp (ns): the per-edge integrated value is bounded to `±e_i_ns`.
    pub e_i_ns: i64,
    /// Back-calculation gain (inverse): on trim saturation, bleed `excess / kbc_inv` back.
    pub kbc_inv: i64,
    /// Lock window/hold, outlier threshold/max, trim clamp — as the production loop.
    pub lock_ns: i64,
    pub lock_hold: u32,
    pub outlier_ns: i64,
    pub outlier_max: u32,
    pub trim_max_mppb: i64,
}

impl IntegralReworkConfig {
    /// Default tuning (the host-experiment settings).
    pub const DEFAULT: Self = Self {
        kp_inv: 8,
        d_den: 4,
        i_den: 128,
        e_i_ns: 800,
        kbc_inv: 5, // ≈ 0.2
        lock_ns: 1_000,
        lock_hold: 5,
        outlier_ns: 3_000,
        outlier_max: 12,
        trim_max_mppb: 3_000_000,
    };
}

impl Default for IntegralReworkConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Flag-free continuous-integral PID + Smith. See the [module docs](self) and
/// [`IntegralReworkConfig`].
#[derive(Debug)]
pub struct IntegralRework {
    cfg: IntegralReworkConfig,
    trim_mppb: i64,
    lock_cnt: u32,
    reject_cnt: u32,
    last_pred: i64,
    last_pd: i64,
}

impl IntegralRework {
    /// Create with the default tuning.
    pub const fn new() -> Self {
        Self::with_config(IntegralReworkConfig::DEFAULT)
    }

    /// Create with a given tuning.
    pub const fn with_config(cfg: IntegralReworkConfig) -> Self {
        Self {
            cfg,
            trim_mppb: 0,
            lock_cnt: 0,
            reject_cnt: 0,
            last_pred: 0,
            last_pd: 0,
        }
    }
}

impl Default for IntegralRework {
    fn default() -> Self {
        Self::new()
    }
}

impl PhaseController for IntegralRework {
    fn step(&mut self, input: ControlInput) -> ControlOutput {
        let ctrl = input.err_ns;
        let pred = ctrl - self.last_pd; // Smith predictor
        let locked = self.lock_cnt >= self.cfg.lock_hold;
        let mut p_corr = 0;
        let mut d_corr = 0;
        let mut applied = false;
        let mut rejected = false;

        if input.valid {
            if locked && ctrl.abs() > self.cfg.outlier_ns && self.reject_cnt < self.cfg.outlier_max {
                self.reject_cnt += 1;
                rejected = true;
            } else {
                self.reject_cnt = 0;
                applied = true;
                // Continuous integral: always integrate, but clamp the integral *input* so a large
                // transient cannot wind it up (this replaces the I-enable gate), then back-calculate
                // any trim saturation so the integrator stays consistent with the actuator.
                let pin = pred.clamp(-self.cfg.e_i_ns, self.cfg.e_i_ns);
                // `.max(1)` on every divisor: integer divide-by-zero panics on the no_std firmware,
                // and these denominators are a public, settable API (no in-tree caller passes 0, but
                // the precondition must be enforced structurally, not just documented).
                let want = self.trim_mppb - pin * 1000 / self.cfg.i_den.max(1);
                let clamped = want.clamp(-self.cfg.trim_max_mppb, self.cfg.trim_max_mppb);
                self.trim_mppb = clamped - (want - clamped) / self.cfg.kbc_inv.max(1);
                p_corr = (pred / self.cfg.kp_inv.max(1)).clamp(-CORR_CLAMP_NS, CORR_CLAMP_NS);
                if locked {
                    d_corr = ((pred - self.last_pred) / self.cfg.d_den.max(1))
                        .clamp(-CORR_CLAMP_NS, CORR_CLAMP_NS);
                }
                self.lock_cnt = if ctrl.abs() < self.cfg.lock_ns {
                    (self.lock_cnt + 1).min(self.cfg.lock_hold)
                } else {
                    0
                };
            }
        }
        if applied {
            self.last_pred = pred;
            self.last_pd = p_corr + d_corr;
        }
        ControlOutput {
            trim_mppb: self.trim_mppb,
            pcorr_ns: p_corr + d_corr,
            applied,
            locked,
            rejected,
            dbg: ControlDebug {
                pred_ns: pred,
                p_ns: p_corr,
                d_ns: d_corr,
                observer_freq_mppb: self.trim_mppb,
                state: 0,
            },
        }
    }

    fn is_locked(&self) -> bool {
        self.lock_cnt >= self.cfg.lock_hold
    }

    fn start_segment(&mut self, init: ControlInit) {
        self.trim_mppb = init.residual_trim_mppb;
        self.lock_cnt = 0;
        self.reject_cnt = 0;
        self.last_pred = 0;
        self.last_pd = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn drive<C: PhaseController>(c: &mut C, err: i64, valid: bool) -> ControlOutput {
        c.step(ControlInput { err_ns: err, valid })
    }

    #[test]
    fn open_loop_emits_no_phase_correction_and_holds_trim() {
        let mut c = OpenLoopFf::new();
        c.start_segment(ControlInit {
            residual_trim_mppb: 5_000,
        });
        for err in [50_000, 0, -800, 1_234] {
            let o = drive(&mut c, err, true);
            assert_eq!(o.pcorr_ns, 0); // never servos phase
            assert_eq!(o.trim_mppb, 5_000); // holds the seeded feedforward residual
            assert!(!o.locked);
        }
        // Invalid edge is not "applied" but still reports the held trim.
        let o = drive(&mut c, 999, false);
        assert!(!o.applied);
        assert_eq!(o.trim_mppb, 5_000);
    }

    #[test]
    fn alpha_beta_locks_from_offset() {
        let mut c = AlphaBetaBoost::new();
        // Closed-loop plant: out starts at a 20 µs offset, controller drives it to zero.
        let mut out: i64 = 20_000;
        let mut locked_at = None;
        for k in 0..400 {
            let o = drive(&mut c, out, true);
            out += o.trim_mppb / 1000 - o.pcorr_ns;
            if c.is_locked() && locked_at.is_none() {
                locked_at = Some(k);
            }
        }
        assert!(locked_at.is_some(), "should lock within 400 edges");
        assert!(out.abs() < 1_000, "should settle inside the lock window, got {out}");
    }

    #[test]
    fn alpha_beta_boost_enters_on_transient_not_on_steady_wander() {
        let mut c = AlphaBetaBoost::new();
        // Settle at zero.
        let mut out: i64 = 0;
        for _ in 0..40 {
            let o = drive(&mut c, out, true);
            out += o.trim_mppb / 1000 - o.pcorr_ns;
        }
        assert!(c.is_locked());
        // Steady wander of several hundred ns (within innov_enter) must NOT boost.
        for err in [300, -250, 400, -350, 200, -300, 350, -400] {
            let o = drive(&mut c, err, true);
            assert_eq!(o.dbg.state, 0, "steady wander {err}ns must not enter boost");
        }
        // A large offset during *reacquisition* (fresh segment → unlocked, so the single-spike
        // outlier guard — which only protects the locked loop — does not apply) is believed as a real
        // transient and enters boost immediately. (While locked, one big spike is instead rejected;
        // a *persistent* one boosts after outlier_max — see the spike test.)
        c.start_segment(ControlInit {
            residual_trim_mppb: 0,
        });
        let o = drive(&mut c, 20_000, true);
        assert_eq!(o.dbg.state, 1, "a reacquisition transient must enter boost");
    }

    #[test]
    fn alpha_beta_start_segment_keeps_output_frequency_continuous() {
        // Regression for the start_segment phantom-innovation defect: the FIRST EMITTED trim after a
        // switch must equal the seed (not just the getter), with pcorr 0 and no boost — otherwise the
        // controller injects a self-made phase excursion at every switch that the others don't, biasing
        // a cross-controller comparison. Check across magnitudes incl. near the trim clamp (where the
        // old truncation-induced ratchet was ~47 ppb/edge).
        for seed in [-7_000, 0, 12_345, 2_999_999, -2_999_999] {
            let mut c = AlphaBetaBoost::new();
            c.start_segment(ControlInit {
                residual_trim_mppb: seed,
            });
            assert_eq!(c.freq_trim_mppb(), seed, "getter before first edge");
            assert!(!c.is_locked());
            let o = drive(&mut c, 0, true); // steady (err==0) first edge
            assert_eq!(o.trim_mppb, seed, "first emitted trim must equal the seed (seed={seed})");
            assert_eq!(o.pcorr_ns, 0, "no phase kick on the first post-switch edge");
            assert_eq!(o.dbg.state, 0, "not boosting on a steady switch");
        }
    }

    #[test]
    fn alpha_beta_boost_does_not_latch_on_periodic_outliers() {
        // Regression for the phantom-boost latch: a periodic outlier (a spike, then one in-window
        // edge, repeating) must NOT pin the boost on — the in-window edge is not a disturbance, and the
        // boost_max_edges backstop must keep ticking through rejected edges. Drive the pattern and
        // require the boost duty stays low (well under half), not the ~82% the defect produced.
        let mut c = AlphaBetaBoost::new();
        let mut out: i64 = 0;
        for _ in 0..40 {
            let o = drive(&mut c, out, true);
            out += o.trim_mppb / 1000 - o.pcorr_ns;
        }
        assert!(c.is_locked());
        let mut boosting = 0;
        let n = 600;
        for k in 0..n {
            // spike past outlier_ns most edges, a clean in-window edge every 13th.
            let err = if k % 13 == 12 { 0 } else { 50_000 };
            let o = drive(&mut c, err, true);
            boosting += o.dbg.state as usize;
        }
        assert!(
            boosting * 2 < n,
            "boost must not latch on periodic outliers (boosting {boosting}/{n} edges)"
        );
    }

    #[test]
    fn controllers_do_not_panic_on_zero_denominator_config() {
        // Regression for the divide-by-zero hardening: zero-valued (public) config denominators must
        // not panic the no_std core. Each `.step` with a zero gain/denominator must return.
        let mut a = AlphaBetaBoost::with_config(AlphaBetaBoostConfig {
            steady_alpha_inv: 0,
            steady_beta_inv: 0,
            steady_pgain_inv: 0,
            fast_alpha_inv: 0,
            fast_beta_inv: 0,
            fast_pgain_inv: 0,
            ..AlphaBetaBoostConfig::DEFAULT
        });
        let _ = drive(&mut a, 100, true);
        let mut i = IntegralRework::with_config(IntegralReworkConfig {
            i_den: 0,
            kbc_inv: 0,
            kp_inv: 0,
            d_den: 0,
            ..IntegralReworkConfig::DEFAULT
        });
        let _ = drive(&mut i, 0, true); // the sharp kbc_inv 0/0 first-edge case
        let _ = drive(&mut i, 500, true);
    }

    #[test]
    fn alpha_beta_rejects_isolated_spike_then_accepts_persistent() {
        let mut c = AlphaBetaBoost::new();
        let mut out: i64 = 0;
        for _ in 0..40 {
            let o = drive(&mut c, out, true);
            out += o.trim_mppb / 1000 - o.pcorr_ns;
        }
        assert!(c.is_locked());
        // Isolated spikes are held (not applied), up to outlier_max.
        for _ in 0..c.config().outlier_max {
            let o = drive(&mut c, 50_000, true);
            assert!(o.rejected && !o.applied);
            assert_eq!(o.pcorr_ns, 0);
        }
        // The next one is accepted as a real disturbance and enters boost.
        let o = drive(&mut c, 50_000, true);
        assert!(!o.rejected && o.applied);
        assert_eq!(o.dbg.state, 1);
    }

    #[test]
    fn integral_rework_locks_and_holds_on_invalid() {
        let mut c = IntegralRework::new();
        let mut out: i64 = 10_000;
        for _ in 0..400 {
            let o = drive(&mut c, out, true);
            out += o.trim_mppb / 1000 - o.pcorr_ns;
        }
        assert!(c.is_locked());
        assert!(out.abs() < 1_000, "should settle, got {out}");
        let trim = drive(&mut c, out, true).trim_mppb;
        let o = drive(&mut c, 999_999, false); // holdover
        assert!(!o.applied);
        assert_eq!(o.pcorr_ns, 0);
        assert_eq!(o.trim_mppb, trim);
        assert!(c.is_locked());
    }

    #[test]
    fn controller_enum_dispatches_and_switches_fairly() {
        use crate::{PhaseLockLoop, PhaseLockLoopConfig};
        // Cycle the enum through every variant in one "boot", switching with the fair-segment
        // protocol. Each closed-loop variant must lock from a fresh offset; the switch must seed the
        // residual trim (output frequency continuous) and blank lock (segment starts in warmup).
        let mut list = [
            Controller::Pll(PhaseLockLoop::new()),
            Controller::IntegRework(IntegralRework::new()),
            Controller::AbBoost(AlphaBetaBoost::new()),
            Controller::Pll(PhaseLockLoop::with_config(PhaseLockLoopConfig::NAIVE_PID)),
        ];
        for c in list.iter_mut() {
            // Fair start: seed a residual trim; the controller must report it (continuous freq) and
            // be unlocked (warmup) immediately after the switch.
            c.start_segment(ControlInit {
                residual_trim_mppb: 4_321,
            });
            assert!(!c.is_locked(), "{} should start a segment unlocked", c.name());
            // Drive a 15 µs offset closed-loop; every variant here is a phase servo → must lock.
            let mut out: i64 = 15_000;
            for _ in 0..400 {
                let o = c.step(ControlInput {
                    err_ns: out,
                    valid: true,
                });
                out += o.trim_mppb / 1000 - o.pcorr_ns;
            }
            assert!(c.is_locked(), "{} should lock after switch", c.name());
            assert!(out.abs() < 1_000, "{} should settle, got {out}", c.name());
        }
        // The open-loop variant dispatches but never servos/locks.
        let mut ol = Controller::OpenLoop(OpenLoopFf::new());
        let o = ol.step(ControlInput {
            err_ns: 9_999,
            valid: true,
        });
        assert_eq!(o.pcorr_ns, 0);
        assert!(!ol.is_locked());
        assert_eq!(ol.name(), "open_loop");
    }

    #[test]
    fn integral_rework_integral_input_is_clamped() {
        // A huge sustained error must not wind the integrator at the full rate: the per-edge step is
        // bounded by e_i_ns, not by the (much larger) error.
        let mut c = IntegralRework::new();
        let big = drive(&mut c, 100_000, true).trim_mppb.abs();
        let cfg = IntegralReworkConfig::DEFAULT;
        // Bound: |trim step| ≤ e_i·1000/i_den + back-calc slack (no saturation here).
        let bound = cfg.e_i_ns * 1000 / cfg.i_den;
        assert!(big <= bound as i64, "first step {big} should be clamped to ≤ {bound}");
    }
}
