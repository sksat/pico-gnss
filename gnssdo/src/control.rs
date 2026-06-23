//! A pluggable output-phase **controller** abstraction, so multiple discipline strategies can be
//! implemented and compared under matched conditions (same boot, same reception).
//!
//! Why an abstraction at all: the loop measures output-vs-**the same receiver** (`hwphase`), so a
//! steady-state "closer to UTC" claim can't be verified without an external reference (the
//! *measurement trap*; see `report/REPORT.md`). What *can* be verified reception-independently is the
//! closed-loop response, by cycling controllers in one boot and injecting a known PRBS into each.
//! That comparison is only fair if every controller is the *same kind of object* on the *same plant
//! input*. Hence this trait: each controller is a pure phase servo that consumes a measured phase
//! error and emits the same two actuator values as [`PhaseLockLoop`](crate::PhaseLockLoop) — a
//! **residual** frequency trim (milli-ppb, added on top of the crystal feedforward) and an immediate
//! phase correction (ns).
//!
//! Two things deliberately live **outside** the controller:
//! - the crystal-frequency **feedforward** ([`DisciplinedClock`](crate::DisciplinedClock)) — the
//!   controller returns the trim *on top of* it, and any internal frequency estimate is the
//!   *residual* after feedforward, never absolute. Keeping the feedforward identical for every
//!   controller is what makes the comparison about the phase servo and not a different whole system.
//! - the **PRBS** identification signal — it is a plant-input disturbance added *after* the
//!   controller, so the controller responds to it. Putting it in the trait would let a controller
//!   "see" the PRBS and cancel it, contaminating the identification.

/// One output edge's input to a [`PhaseController`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ControlInput {
    /// Measured output-vs-reference phase error (ns), e.g. from a hardware loopback capture.
    pub err_ns: i64,
    /// Whether this sample is trustworthy (reference fresh, interval sane). When false the controller
    /// holds its state and emits no correction (holdover).
    pub valid: bool,
}

/// Per-controller debug/telemetry. Logged for analysis; the caller's actuation uses only
/// [`ControlOutput::trim_mppb`] and [`ControlOutput::pcorr_ns`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ControlDebug {
    /// The control input the strategy actually acted on (e.g. the Smith-predicted phase), ns.
    pub pred_ns: i64,
    /// Proportional-equivalent component of `pcorr_ns` (ns).
    pub p_ns: i64,
    /// Derivative-equivalent component of `pcorr_ns` (ns).
    pub d_ns: i64,
    /// The controller's internal **residual** frequency estimate (milli-ppb) if it keeps one, else 0.
    /// Residual = the frequency left after the external feedforward — never the absolute crystal
    /// offset (that is the feedforward's job, kept identical across controllers).
    pub observer_freq_mppb: i64,
    /// Strategy-specific state code (e.g. a transient-boost state machine's phase). 0 = nominal.
    pub state: u8,
}

/// Result of [`PhaseController::step`]. The caller forms the output period from `trim_mppb` (added to
/// the crystal feedforward) and `pcorr_ns`; the rest is telemetry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ControlOutput {
    /// **Residual** frequency trim (milli-ppb) to add to the crystal feedforward.
    pub trim_mppb: i64,
    /// Immediate phase correction (ns) to subtract from this period.
    pub pcorr_ns: i64,
    /// Whether this edge was applied (false if invalid or rejected as an outlier).
    pub applied: bool,
    /// Lock state used for this edge's control decisions.
    pub locked: bool,
    /// Whether this edge was held as an outlier (no correction).
    pub rejected: bool,
    /// Telemetry (not used for actuation).
    pub dbg: ControlDebug,
}

/// Fair start-of-segment seed for switching controllers mid-run (same-boot cycling).
///
/// To compare methods without mixing in re-acquisition, the caller makes the output frequency
/// **continuous** across the switch: it hands the next controller the residual trim that keeps
/// `feedforward + trim` unchanged, and every per-edge history (Smith / derivative / observer / boost
/// / lock counter) is blanked. The first post-switch `pcorr_ns` then starts from 0, so there is no
/// spurious derivative kick. See `report/REPORT.md`'s cross-controller comparison protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ControlInit {
    /// Initial residual frequency trim (milli-ppb) = `applied_freq − new_feedforward`, so the output
    /// frequency does not jump at the switch.
    pub residual_trim_mppb: i64,
}

/// A pluggable output-phase discipline strategy. Implementors are integer-only / `no_std` and behave
/// as a pure phase servo on top of the shared crystal feedforward (see the [module docs](self)).
///
/// The method is named `step` (not `update`) to match the host comparison harness's vocabulary and
/// to avoid colliding with a type's own richer inherent `update` (e.g. [`PhaseLockLoop`] keeps its
/// detailed `update` for the firmware/tests and implements `step` by delegating to it).
pub trait PhaseController {
    /// Process one output edge and return the two actuator values plus telemetry.
    fn step(&mut self, input: ControlInput) -> ControlOutput;
    /// Whether the controller is currently locked.
    fn is_locked(&self) -> bool;
    /// Re-seed for a fresh comparison segment (a fair cross-controller switch).
    fn start_segment(&mut self, init: ControlInit);
}
