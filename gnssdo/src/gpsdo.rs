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
/// (`Debug`/`Default` is the minimum required by [`DisciplinedClock`]'s derive; `Clone`/`Copy`/
///  `PartialEq` etc. are not added until actually needed.)
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
    /// **Temperature feedforward** of the crystal frequency. When `true`, [`update_temp`] +
    /// [`predicted_freq_mppb`] use an online-learned model `freq ≈ base + k·(temp − ref)` to predict
    /// the crystal frequency from temperature with **no EMA lag**, instead of the α-β level. On a bare
    /// crystal the medium-term (64–300 s) frequency wander — the dominant output-phase wander — is
    /// largely temperature-driven and ~80 % predictable from the on-chip sensor (verified out-of-sample
    /// on real logs), so feeding it forward removes the lag-induced wander before the phase loop sees
    /// it. Default `false` (α-β only; production unchanged until a temp source is wired + validated).
    pub temp_ff_enable: bool,
    /// EMA shift for the online temperature↔frequency regression (means + covariance/variance). The
    /// coefficient `k = cov(temp,freq)/var(temp)` is learned with this smoothing; larger = slower,
    /// more stable adaptation. Scale-robust (the variance normalizes `k`), so it works regardless of
    /// the temperature span. Default 7 (≈128-sample). The crystal's temp coefficient is nonlinear, so
    /// learning it online tracks the local slope.
    pub temp_ff_shift: u32,
    /// **Unused in the temperature-feedforward path** (kept only so old sweeps/configs that set it
    /// still compile). The previous ad-hoc design blended a lead term `λ·k·(temp_sm − temp_lvl)` at
    /// authority `λ` (Q8, 256 = 1.0) to hedge a noisily-learned `k`. The matched-lead design supersedes
    /// it: the die→crystal thermal lag is compensated structurally (see
    /// [`temp_ff_lag_q8`](Self::temp_ff_lag_q8)) so the feedforward runs at full (1.0) authority, and a
    /// fixed-gain residual observer ([`resid_obs_shift`](Self::resid_obs_shift)) — not a haircut —
    /// absorbs whatever temperature does not explain. `k` runaway is instead bounded by
    /// [`temp_k_clamp_mppb`](Self::temp_k_clamp_mppb).
    pub temp_ff_gain_q8: i64,
    /// EMA shift that pre-smooths the raw temperature before it enters the regression **and** the
    /// matched-lead low-pass (`temp_die_lp`). The on-chip sensor is coarsely quantized (≈1 raw step)
    /// and ADC-noisy; multiplying that noise by the temperature coefficient `k` (tens of ppb/raw) would
    /// inject step disturbances into the feedforward and, via errors-in-variables, bias `k` low (noise
    /// inflates `var(temp)`). Smoothing the temperature at a shorter timescale than the long-term mean
    /// (`temp_ff_shift`) removes the ADC noise while keeping the slow temperature-driven crystal wander
    /// (64–300 s). Default 4 (≈16-sample); must be `< temp_ff_shift` or the deviation term collapses.
    pub temp_ff_sm_shift: u32,
    /// Matched-lead gain `τ/Ts` in Q8 fixed-point (256 = 1.0), default 1536 (≈6.0, i.e. τ≈6 s at the
    /// 1 s loop rate). The on-chip die sensor leads the crystal by a 1st-order thermal lag τ, so the
    /// crystal temperature estimate is `Tcrys_hat = die_lp + (τ/Ts)·Δdie_lp` — a discrete lead
    /// compensator that cancels that lag (and the slow control loop's tracking lag) to first order,
    /// instead of the previous noisy slope projection. 0 = no lead (low-passed die temperature only).
    pub temp_ff_lag_q8: i64,
    /// EMA shift that band-limits the die-temperature derivative feeding the matched lead (default 2 =
    /// α 1/4). The lead multiplies the per-sample temperature *difference* by `τ/Ts`; differentiating
    /// raw `temp_die_lp` would amplify residual ADC noise by that factor, so the difference is smoothed
    /// before it is scaled. 0..=63 (clamped).
    pub temp_ff_dlead_shift: u32,
    /// Fixed observer gain `K = 1/2^resid_obs_shift` for the non-thermal residual frequency `r_resid`
    /// (default 2 = K 0.25). Each locked edge updates `r_resid += (measured − f_ff − r_resid) >> shift`,
    /// a constant-gain Kalman filter on the part of the frequency the temperature model does not
    /// explain (crystal aging, supply, the DC offset). The feedforward then predicts `f_ff + r_resid`.
    /// Smaller shift = faster but jitterier; 0..=63 (clamped).
    pub resid_obs_shift: u32,
    /// Loose clamp on the learned temperature coefficient `|k| = |cov/var|` in milli-ppb per **raw**
    /// temperature unit (default 100 000 = 100 ppb/raw). Replaces the `λ` haircut as the runaway guard:
    /// with matched-lead + observer the feedforward runs at full authority, so a transiently bad `k`
    /// (cold-start, near-zero temperature span) must not drive the feedforward wild. The default is far
    /// above any plausible crystal TC (≈0.5 ppm/°C) so it only catches blow-ups, not normal learning.
    pub temp_k_clamp_mppb: i64,
    /// Half-width (raw temperature units, default 64) of the clamp that bounds `Tcrys_hat` to
    /// `temp_die_lp ± this`. The matched lead can spike if a single ADC glitch slips through the
    /// derivative band-limit; clamping the crystal-temperature estimate around the low-passed die
    /// temperature caps that transient. Generous enough not to clip a normal thermal ramp's lead (a few
    /// raw units), so it is a guard, not a regular limiter. Scale it up with a finer (oversampled)
    /// temperature source whose raw unit is smaller.
    pub temp_ff_lead_clamp_raw: i64,
    /// Clamp (milli-ppb, default 100 000 = 100 ppb) on the feedforward **deviation** handed to the
    /// output steering — `steering_freq_mppb() = freq_mppb + clamp(predicted − freq_mppb, ±this)`.
    /// In a fast thermal transient the feedforward `predicted` over-reacts (on hardware ≈ −1300 ppb
    /// while the settled change was ≈ −0.8 ppb), which would slam the output period; bounding the
    /// steered deviation to a physically plausible range absorbs that spike. It does **not** touch
    /// [`predicted_freq_mppb`](DisciplinedClock::predicted_freq_mppb) (holdover keeps the raw value).
    /// The legitimate feedforward is small (α-β slope `slope_max·pred_lead` = 5 ppb; the matched-lead
    /// caps at ~41 ppb in the host model with k=4 ppb/raw), so 100 ppb leaves normal operation
    /// untouched.
    ///
    /// The 100-ppb default is **provisional**: the host model internally clamps the matched-lead FF and
    /// does **not** reproduce the hardware −1300 ppb over-reaction (likely the `r_resid` observer
    /// spiking on die↔crystal model mismatch during fast heating), so the bound and its actual effect
    /// must be tuned and validated on hardware, not from the host plant.
    pub steer_ff_bound_mppb: i64,
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
        temp_ff_enable: false,
        temp_ff_shift: 7,
        temp_ff_sm_shift: 4,
        temp_ff_gain_q8: 64,
        temp_ff_lag_q8: 1536,
        temp_ff_dlead_shift: 2,
        resid_obs_shift: 2,
        temp_k_clamp_mppb: 100_000,
        temp_ff_lead_clamp_raw: 64,
        steer_ff_bound_mppb: 100_000,
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
    // Temperature feedforward (config.temp_ff_enable), state-space form (Ts = 1 s):
    //   f_ff[k] = k·(Tcrys_hat − temp_ref)   (known input; k = temp_cov/temp_var)
    //   r_resid = non-thermal residual frequency (random walk; fixed-gain observer)
    //   output  = f_ff + r_resid
    // The matched lead estimates the crystal temperature from the die sensor; the online regression
    // (temp_sm/temp_mt/temp_mf_mppb/temp_cov/temp_var) learns the local TC `k`.
    temp_now: i64,      // latest temperature reading (raw sensor units), via update_temp
    temp_have: bool,    // a temperature has been observed
    temp_sm: i64,       // short EMA of temperature (<<TEMP_FP), ADC-noise removed; regression input
    temp_mt: i64,       // long EMA of temperature (<<TEMP_FP), the centering mean
    temp_mf_mppb: i64,  // EMA of measured frequency (milli-ppb)
    temp_cov: i64,      // EMA of (temp_sm−mt)·(freq−mf)
    temp_var: i64,      // EMA of (temp_sm−mt)²
    temp_ref: i64,      // reference temperature (first reading, <<TEMP_FP); f_ff is k·(Tcrys_hat−ref)
    r_resid: i64,       // non-thermal residual frequency estimate (milli-ppb), observer state
    // Matched-lead state (die temperature → crystal temperature). Kept separate from temp_sm so the
    // lead path can later run at a higher rate (e.g. 30 Hz) than the 1 Hz regression; today they share
    // temp_ff_sm_shift so temp_die_lp mirrors temp_sm.
    temp_die_lp: i64,   // low-passed die temperature (<<TEMP_FP)
    temp_die_prev: i64, // previous temp_die_lp, for the per-sample difference
    temp_dlead: i64,    // band-limited derivative of temp_die_lp (<<TEMP_FP per sample)
    tcrys_hat: i64,     // estimated crystal temperature (<<TEMP_FP) = die_lp + lead, clamped
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

/// Fixed-point fractional bits for the temperature-feedforward EMA states. Temperature is a
/// small-range integer, so its EMAs are kept in `<<TEMP_FP` units to keep the `>>shift` increment of a
/// 1-unit change from rounding to zero.
const TEMP_FP: u32 = 8;

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
            temp_now: 0,
            temp_have: false,
            temp_sm: 0,
            temp_mt: 0,
            temp_mf_mppb: 0,
            temp_cov: 0,
            temp_var: 0,
            temp_ref: 0,
            r_resid: 0,
            temp_die_lp: 0,
            temp_die_prev: 0,
            temp_dlead: 0,
            tcrys_hat: 0,
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
        // Temperature feedforward (state-space form). Two coupled estimators, both only while locked:
        //   1. Online regression of the *instantaneous* measured frequency (`measured_mppb`, not the
        //      smoothed level) on temperature (means + covariance/variance EMAs) learns the local TC
        //      `k = cov/var`. Regressing the level would only reproduce its EMA lag; regressing the
        //      instantaneous measurement lets `k` reconstruct the temperature-driven frequency with no
        //      lag, and PPS jitter, uncorrelated with temperature, averages out of the covariance.
        //   2. A fixed-gain observer on the *non-thermal* residual `r_resid`: the innovation is the
        //      measured frequency minus the matched-lead feedforward `f_ff` (evaluated at this edge's
        //      crystal-temperature estimate) and the current residual. `r_resid` absorbs the part the
        //      temperature cannot explain (DC offset, aging, supply), so the prediction `f_ff+r_resid`
        //      tracks the true frequency with no EMA lag.
        if self.config.temp_ff_enable && self.temp_have && self.is_locked() {
            // Temperature is a small-range integer; keep its EMA states in fixed-point (`<<TEMP_FP`) so
            // the `>>shift` EMA increment of a 1-unit temperature change does not round to zero.
            let s = self.config.temp_ff_shift.min(30);
            self.temp_mt += (self.temp_sm - self.temp_mt) >> s;
            self.temp_mf_mppb += (measured_mppb - self.temp_mf_mppb) >> s;
            let dt = self.temp_sm - self.temp_mt; // smoothed fixed-point temperature residual
            let df = measured_mppb - self.temp_mf_mppb;
            self.temp_cov += (dt * df - self.temp_cov) >> s;
            self.temp_var += (dt * dt - self.temp_var) >> s;
            // Residual observer: r_resid += K·(measured − f_ff_at_meas − r_resid), K = 1/2^shift.
            // f_ff uses this edge's crystal-temperature estimate (the measurement time).
            let f_ff_meas = self.temp_ff_mppb(self.tcrys_hat);
            let innov = measured_mppb - f_ff_meas - self.r_resid;
            let obs = self.config.resid_obs_shift.min(63);
            self.r_resid += innov >> obs;
        }
        self.samples += 1;
        FreqUpdate::Applied
    }

    /// Feed a temperature reading (raw sensor units) for the temperature feedforward
    /// ([`DisciplinedClockConfig::temp_ff_enable`]). Call once per edge alongside
    /// [`update_freq`](Self::update_freq). The first reading sets the model's reference. No-op for the
    /// model unless `temp_ff_enable` is set, but always records the latest value.
    pub fn update_temp(&mut self, temp: i64) {
        self.temp_now = temp;
        let t_fp = temp << TEMP_FP; // fixed-point temperature
        if !self.temp_have {
            self.temp_sm = t_fp;
            self.temp_mt = t_fp;
            self.temp_mf_mppb = self.freq_mppb;
            // Matched-lead seed: reference and lead state at the first temperature (zero lead/deriv).
            self.temp_ref = t_fp;
            self.temp_die_lp = t_fp;
            self.temp_die_prev = t_fp;
            self.temp_dlead = 0;
            self.tcrys_hat = t_fp;
            self.temp_have = true;
        } else {
            // Short EMA to strip the ADC quantization/noise before it is fed forward (×k).
            let sm = self.config.temp_ff_sm_shift.min(30);
            self.temp_sm += (t_fp - self.temp_sm) >> sm;
            // Matched lead: estimate the crystal temperature from the die sensor. Low-pass the die
            // temperature (same band-limit as the regression input), take its band-limited per-sample
            // derivative, and advance by τ/Ts to cancel the die→crystal thermal lag (and the slow loop
            // lag) to first order: Tcrys_hat = die_lp + (τ/Ts)·d(die_lp).
            self.temp_die_lp += (t_fp - self.temp_die_lp) >> sm;
            let raw_d = self.temp_die_lp - self.temp_die_prev;
            self.temp_die_prev = self.temp_die_lp;
            let dl = self.config.temp_ff_dlead_shift.min(63);
            self.temp_dlead += (raw_d - self.temp_dlead) >> dl;
            let lead = (self.config.temp_ff_lag_q8 * self.temp_dlead) >> 8;
            // Clamp the estimate around the low-passed die temperature to cap a transient glitch spike.
            let clamp = self.config.temp_ff_lead_clamp_raw << TEMP_FP;
            self.tcrys_hat =
                (self.temp_die_lp + lead).clamp(self.temp_die_lp - clamp, self.temp_die_lp + clamp);
        }
    }

    /// Matched-lead feedforward `f_ff = k·(Tcrys_hat − temp_ref)` in milli-ppb, with `k = cov/var`
    /// clamped to the physical range ([`temp_k_clamp_mppb`](DisciplinedClockConfig::temp_k_clamp_mppb)).
    /// 0 until the temperature has varied (var > 0). `tcrys_hat_fp` is a crystal-temperature estimate
    /// in `<<TEMP_FP` fixed-point.
    fn temp_ff_mppb(&self, tcrys_hat_fp: i64) -> i64 {
        if self.temp_var <= 0 {
            return 0;
        }
        let clamp = self.config.temp_k_clamp_mppb;
        let k = self.temp_k_mppb_per_unit().clamp(-clamp, clamp); // milli-ppb per raw unit
        let dt = tcrys_hat_fp - self.temp_ref; // <<TEMP_FP
        (k as i128 * dt as i128 / (1i128 << TEMP_FP)) as i64
    }

    /// Toggle the temperature feedforward at runtime (sets [`DisciplinedClockConfig::temp_ff_enable`]).
    pub fn set_temp_ff_enable(&mut self, en: bool) {
        self.config.temp_ff_enable = en;
    }

    /// Tune the temperature feedforward at runtime (sets the three temp-FF knobs in
    /// [`DisciplinedClockConfig`]): `sm_shift` = short EMA that strips ADC noise from the temperature
    /// before it is fed forward ([`temp_ff_sm_shift`](DisciplinedClockConfig::temp_ff_sm_shift)),
    /// `shift` = the regression mean/covariance EMA ([`temp_ff_shift`](DisciplinedClockConfig::temp_ff_shift)),
    /// `gain_q8` = the lead authority in Q8 ([`temp_ff_gain_q8`](DisciplinedClockConfig::temp_ff_gain_q8)).
    pub fn set_temp_ff_params(&mut self, sm_shift: u32, shift: u32, gain_q8: i64) {
        self.config.temp_ff_sm_shift = sm_shift;
        self.config.temp_ff_shift = shift;
        self.config.temp_ff_gain_q8 = gain_q8;
    }

    /// Tune the matched-lead + residual-observer knobs at runtime: `lag_q8` = the lead gain `τ/Ts` in
    /// Q8 ([`temp_ff_lag_q8`](DisciplinedClockConfig::temp_ff_lag_q8)), `dlead_shift` = the derivative
    /// band-limit ([`temp_ff_dlead_shift`](DisciplinedClockConfig::temp_ff_dlead_shift)), `obs_shift` =
    /// the residual observer gain ([`resid_obs_shift`](DisciplinedClockConfig::resid_obs_shift)).
    pub fn set_temp_ff_lag(&mut self, lag_q8: i64, dlead_shift: u32, obs_shift: u32) {
        self.config.temp_ff_lag_q8 = lag_q8;
        self.config.temp_ff_dlead_shift = dlead_shift;
        self.config.resid_obs_shift = obs_shift;
    }

    /// Learned temperature coefficient `k = cov(temp,freq)/var(temp)` in milli-ppb per **raw**
    /// temperature unit, 0 until temperature has varied. For logging/diagnostics. (The internal EMAs
    /// keep temperature in `<<TEMP_FP` fixed-point; this getter rescales to raw units.)
    pub fn temp_k_mppb_per_unit(&self) -> i64 {
        if self.temp_var > 0 {
            (self.temp_cov << TEMP_FP) / self.temp_var
        } else {
            0
        }
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

    /// Like [`corrected`](Self::corrected), but also projects the α-β frequency **slope** across the
    /// elapsed interval — i.e. assumes the crystal keeps drifting as `level + slope·t` rather than
    /// freezing at the last level. This matters during **holdover**: the level-only `corrected`
    /// leaves a *quadratic* phase error (≈½·slope·t²) under a sustained temperature ramp, because the
    /// frozen level grows staler as the ramp continues. Projecting the slope cancels that to first
    /// order, turning the holdover endpoint error from O(t²) into O(t). Identical to `corrected` when
    /// the slope is 0 (steady state), so it is safe to use unconditionally. The output *period*
    /// already uses [`predicted_freq_mppb`](Self::predicted_freq_mppb) (slope-aware); this brings the
    /// *time read-out* (`now_from_*`, holdover) to parity.
    fn corrected_slope(&self, d: i64) -> i64 {
        // Level term (same as `corrected`): d·level/1e12.
        let level = (d as i128 * self.freq_mppb as i128 / 1_000_000_000_000i128) as i64;
        // Slope term: ∫₀ᵈ (slope·(τ/1e9)) / 1e12 dτ = slope·d² / (2·1e9·1e12). The slope is
        // milli-ppb per *sample* (≈1 s = 1e9 ns), so τ/1e9 converts elapsed ns → samples. i128 keeps
        // d² (up to ~1e24 for a ~1000 s holdover) from overflowing.
        let slope_term = (self.freq_slope_mppb as i128 * d as i128 * d as i128
            / 2_000_000_000_000_000_000_000i128) as i64;
        d - level - slope_term
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
        // Temperature feedforward (state-space): predict the crystal frequency as the matched-lead
        // feedforward at the *latest* crystal-temperature estimate plus the non-thermal residual,
        // `f_ff(Tcrys_hat) + r_resid`. The temperature term has no EMA lag (the matched lead cancels the
        // die→crystal and loop lag), and r_resid carries the directly-measured part temperature cannot
        // explain (DC offset, aging). Falls back to the α-β projection until temperature has varied
        // (var>0) or if disabled — that fallback path is bit-identical to production.
        if self.config.temp_ff_enable && self.temp_have && self.temp_var > 0 {
            return self.temp_ff_mppb(self.tcrys_hat) + self.r_resid;
        }
        self.freq_mppb + self.freq_slope_mppb * self.config.pred_lead_samples
    }

    /// Frequency (milli-ppb) to feed the **output period steering** — the same as
    /// [`predicted_freq_mppb`](Self::predicted_freq_mppb) but with the feedforward *deviation* from the
    /// α-β level bounded to ±[`steer_ff_bound_mppb`](DisciplinedClockConfig::steer_ff_bound_mppb).
    /// `predicted` itself is left raw; only the value handed to the steering is clamped, so a fast
    /// thermal transient where the temperature feedforward `predicted` over-reacts cannot slam the
    /// output period. On hardware a fast hand-heating step drove the matched-lead `predicted` deviation
    /// to ≈ −1300 ppb while the settled crystal change was only ≈ −0.8 ppb (a ~1700× over-reaction),
    /// kicking the output phase by several µs.
    ///
    /// Holdover does **not** go through this getter: the time read-out
    /// ([`now_from_capture_ns_holdover`](Self::now_from_capture_ns_holdover) and `corrected_slope`)
    /// extrapolates from `freq_mppb`/`freq_slope_mppb` directly, so its slope-aware projection is
    /// untouched. The clamp is dormant in normal operation: the α-β-only deviation is ≤
    /// `slope_max·pred_lead` (= 5 ppb), and the matched-lead feedforward caps at ~41 ppb in the host
    /// model, both inside the 100-ppb bound, so this is bit-identical to
    /// `predicted_freq_mppb` until a fast transient pushes the deviation past the bound.
    pub fn steering_freq_mppb(&self) -> i64 {
        let raw = self.predicted_freq_mppb();
        let delta = raw - self.freq_mppb;
        self.freq_mppb
            + delta.clamp(-self.config.steer_ff_bound_mppb, self.config.steer_ff_bound_mppb)
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

    /// Slope-aware variant of [`now_from_capture_ns`](Self::now_from_capture_ns) for **holdover**:
    /// projects the α-β frequency slope across the elapsed interval (see
    /// [`corrected_slope`](Self::corrected_slope)). Use during long PPS outages under thermal drift,
    /// where the level-only read-out accrues a quadratic error. Equals `now_from_capture_ns` in
    /// steady state (slope 0).
    pub fn now_from_capture_ns_holdover(&self, capture_ns: u64) -> Option<i64> {
        let ep = self.epoch_capture_ns?;
        let eu = self.epoch_unix_ns?;
        Some(eu + self.corrected_slope(capture_ns as i64 - ep as i64))
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

    /// Slope-aware variant of [`prediction_residual_ns`](Self::prediction_residual_ns) (holdover):
    /// the disciplined clock's endpoint error when the frequency slope is projected across the gap.
    /// Compare against `prediction_residual_ns` (level-only) to quantify the slope feedforward's
    /// holdover benefit (the holdover-endpoint protocol, which escapes the same-receiver measurement
    /// trap because the endpoint error grows with the gap while the receiver-PPS noise does not).
    pub fn prediction_residual_holdover_ns(&self, capture_ns: u64, unix_ns: i64) -> Option<i64> {
        self.now_from_capture_ns_holdover(capture_ns)
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

    /// Feed a temperature reading (raw sensor units) for the temperature feedforward. Passthrough to
    /// [`DisciplinedClock::update_temp`]; call once per edge alongside the frequency discipline.
    pub fn update_temp(&mut self, temp: i64) {
        self.clock.update_temp(temp);
    }

    /// Toggle the temperature feedforward at runtime. Passthrough to
    /// [`DisciplinedClock::set_temp_ff_enable`].
    pub fn set_temp_ff_enable(&mut self, en: bool) {
        self.clock.set_temp_ff_enable(en);
    }

    /// Tune the temperature feedforward at runtime. Passthrough to
    /// [`DisciplinedClock::set_temp_ff_params`].
    pub fn set_temp_ff_params(&mut self, sm_shift: u32, shift: u32, gain_q8: i64) {
        self.clock.set_temp_ff_params(sm_shift, shift, gain_q8);
    }

    /// Tune the matched-lead + residual-observer knobs at runtime. Passthrough to
    /// [`DisciplinedClock::set_temp_ff_lag`].
    pub fn set_temp_ff_lag(&mut self, lag_q8: i64, dlead_shift: u32, obs_shift: u32) {
        self.clock.set_temp_ff_lag(lag_q8, dlead_shift, obs_shift);
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
    fn temp_ff_holdover_tracks_continuous_temp() {
        // 自己クロック=水晶周波数は連続量で、温度で常に drift する。温度 FF (matched-lead + 残差 observer)
        // の本命価値は holdover: PPS が切れて周波数を直測できなくても、温度センサは生きているので、
        // f_ff = k·(Tcrys_hat − ref) を温度変化に合わせて連続的に動かし、非熱残差 r_resid は凍結して外挿する。
        // ここでは warmup で k と r_resid を学習 → 周波数更新を止め (holdover) → 温度だけ与え続け、温度 FF の
        // 予測 (f_ff + r_resid) が「level 凍結」より真の温度駆動周波数に桁で近いことを確認する。
        let (k_true, f0, tref) = (4000i64, 3_000_000i64, 880i64); // 4 ppb/raw
        let cfg = DisciplinedClockConfig {
            temp_ff_enable: true,
            ..DisciplinedClockConfig::DEFAULT
        };
        let mut c = DisciplinedClock::with_config(cfg);
        let truef = |temp: i64| f0 + k_true * (temp - tref); // mppb
        // warmup: 定常サイクリング温度で k を学習 (周波数も温度で動く)。
        let warm = |n: i64| tref + (10.0 * (n as f64 * core::f64::consts::TAU / 60.0).sin()) as i64;
        for n in 0..1000i64 {
            let t = warm(n);
            c.update_temp(t);
            c.update_freq(1_000_000_000 + truef(t) / 1000);
        }
        let frozen_level = c.freq_mppb(); // holdover で凍結される α-β level
        let base = warm(999);
        // holdover: 周波数更新を止め、温度だけランプさせて与える (連続量)。
        let (mut e_ff, mut e_frozen) = (0i64, 0i64);
        for h in 1..=120i64 {
            let t = base + h; // 温度が holdover 中に 1 raw/s でランプ
            c.update_temp(t);
            let tf = truef(t);
            e_ff += (c.predicted_freq_mppb() - tf).abs();
            e_frozen += (frozen_level - tf).abs();
        }
        // 温度 FF holdover 外挿は level 凍結より終端で桁違いに良い (>2x)。
        assert!(
            e_ff * 2 < e_frozen,
            "temp-aware holdover should beat frozen level: ff={e_ff}mppb frozen={e_frozen}mppb"
        );
        // 学習した係数が真値に収束 (±50%)。
        let k = c.temp_k_mppb_per_unit();
        assert!(
            (k - k_true).abs() < k_true / 2,
            "learned temp coeff should approach truth: k={k} expected~{k_true}"
        );
    }

    #[test]
    fn temp_ff_disabled_is_unchanged_and_falls_back() {
        // 既定 (disabled) では update_temp を呼んでも predicted は α-β のまま (production 不変)。
        // 温度をランプさせても (有効なら f_ff が育つ条件) disabled は α-β path に固定。
        let mut c = DisciplinedClock::new();
        // 温度を呼ばない参照クロック (= production)。同じ周波数列を与える。
        let mut ref_c = DisciplinedClock::new();
        for n in 0..60i64 {
            c.update_temp(900 + n); // 温度をランプ
            c.update_freq(1_000_002_500 + n);
            ref_c.update_freq(1_000_002_500 + n);
        }
        // disabled の predicted は α-β projection そのもの。
        assert_eq!(
            c.predicted_freq_mppb(),
            c.freq_mppb() + c.freq_slope_mppb() * c.config().pred_lead_samples
        );
        // update_temp を呼んだ／呼ばないで predicted が bit 一致 (production 完全不変)。
        assert_eq!(c.predicted_freq_mppb(), ref_c.predicted_freq_mppb());
        assert_eq!(c.freq_mppb(), ref_c.freq_mppb());
        assert_eq!(c.temp_k_mppb_per_unit(), 0); // disabled では回帰を回さない
    }

    #[test]
    fn steering_clamp_bounds_feedforward_on_fast_thermal_transient() {
        // 速い熱過渡では温度FF (matched-lead) が過反応し、predicted_freq_mppb が物理的にありえない量
        // 振れる (実機で実変化の ~1700 倍 = −1300 ppb)。steering_freq_mppb は predicted の生値を
        // holdover 用にそのまま残しつつ、**操舵に渡す FF 偏差** (steering − level) を ±steer_ff_bound_mppb
        // に bound する。ここでは warmup で k・r_resid を学習 → 周波数直測を止め (holdover) → die 温度を
        // 速くランプさせ、predicted は大きく振れてよいが steering の FF 偏差が常に bound 内に収まることを確認。
        let (k_true, f0, tref) = (4000i64, 3_000_000i64, 880i64); // 4 ppb/raw
        // 機構を示すため bound を明示的に小さく取る。host モデルは matched-lead FF を内部で ~41 ppb に
        // clamp してしまい hardware の −1300 ppb を再現しないので、default(100 ppb)では legitimate FF が
        // bound を超えず clamp 発火を示せない。小 bound で「偏差が bound を超える状況で steering が飽和する」
        // 機構を検証する (実機での効果は別途オシロで検証)。
        let cfg = DisciplinedClockConfig {
            temp_ff_enable: true,
            steer_ff_bound_mppb: 5_000,
            ..DisciplinedClockConfig::DEFAULT
        };
        let bound = cfg.steer_ff_bound_mppb;
        let mut c = DisciplinedClock::with_config(cfg);
        let truef = |temp: i64| f0 + k_true * (temp - tref); // mppb
        // warmup: 定常サイクリング温度で k と r_resid を学習。
        let warm = |n: i64| tref + (10.0 * (n as f64 * core::f64::consts::TAU / 60.0).sin()) as i64;
        for n in 0..1000i64 {
            let t = warm(n);
            c.update_temp(t);
            c.update_freq(1_000_000_000 + truef(t) / 1000);
        }
        let level = c.freq_mppb(); // holdover 中は α-β level が凍結される
        let base = warm(999);
        // holdover: 周波数直測を止め、die 温度だけ速くランプさせる。matched-lead の微分項が跳ね、
        // f_ff = k·(Tcrys_hat − ref) が過反応 → predicted が bound を大きく超える。
        let mut saw_overreaction = false;
        for h in 1..=40i64 {
            let t = base + 4 * h; // +4 raw/sample の急ランプ
            c.update_temp(t);
            let predicted = c.predicted_freq_mppb();
            let steering = c.steering_freq_mppb();
            // raw predicted は holdover 用に大きく振れてよい。
            if (predicted - c.freq_mppb()).abs() > bound {
                saw_overreaction = true;
                // 飽和時は steering が level±bound に張り付くこと。単なる |·|≤bound は clamp 定義上
                // 恒真で、FF を 0 に潰す退化実装 (steering=level) も通ってしまうので符号付きで固定する。
                let expect = c.freq_mppb() + if predicted > c.freq_mppb() { bound } else { -bound };
                assert_eq!(
                    steering, expect,
                    "saturated steering must sit at level±bound, not collapse to level"
                );
            }
            // 操舵に渡す FF 偏差は必ず ±bound 内。
            assert!(
                (steering - c.freq_mppb()).abs() <= bound,
                "steering FF must be clamped: steering−level={} bound={}",
                steering - c.freq_mppb(),
                bound
            );
        }
        // この過渡で実際に clamp が必要 (predicted が bound を超える) だったことを確認 = clamp が発火。
        assert!(
            saw_overreaction,
            "fast transient should push predicted beyond the bound so the clamp actually engages"
        );
        // holdover では α-β level は動かない (clamp は steering にだけ作用、level/predicted は不変)。
        assert_eq!(c.freq_mppb(), level);
    }

    #[test]
    fn steering_equals_predicted_on_gentle_ramp() {
        // 緩やかな温度ドリフト相当の周波数ランプ (~2 ppb/s) では、操舵に渡る FF 偏差 (α-β slope·pred_lead =
        // 数 ppb) は bound (100 ppb) 内に収まるので clamp は発火せず、steering == predicted。従来の slope/温度
        // FF 効果 (offset② 補正) を壊さないことの回帰。
        let mut c = DisciplinedClock::new();
        let f0 = 3_000_000i64;
        let r = 2_000i64; // +2 ppb/s ランプ
        let mut n = 0i64;
        for _ in 0..80 {
            c.update_freq(1_000_000_000 + (f0 + r * n) / 1000);
            n += 1;
            if c.is_locked() {
                let predicted = c.predicted_freq_mppb();
                let steering = c.steering_freq_mppb();
                // 緩いランプの FF 偏差は bound 内 (clamp は発火しない)。
                assert!(
                    (predicted - c.freq_mppb()).abs() <= c.config().steer_ff_bound_mppb,
                    "gentle-ramp FF should stay within the bound (clamp must not fire): delta={}",
                    predicted - c.freq_mppb()
                );
                // 発火しない → steering は predicted と完全一致。
                assert_eq!(steering, predicted, "no clamp ⇒ steering equals predicted");
            }
        }
    }

    #[test]
    fn steering_dormant_on_gentle_ramp_with_temp_ff() {
        // production 経路 (temp_ff_enable=true) でも、緩やかな温度ランプでは matched-lead の FF 偏差が
        // bound (100 ppb) 内に収まり clamp は dormant のまま (steering==predicted)。温度FF 本来の効果を
        // 壊していないことの回帰 (test2 は temp_ff off の α-β 経路しか見ておらず production 経路に穴があった)。
        let (k_true, f0, tref) = (4000i64, 3_000_000i64, 880i64); // 4 ppb/raw
        let cfg = DisciplinedClockConfig {
            temp_ff_enable: true,
            ..DisciplinedClockConfig::DEFAULT
        };
        let bound = cfg.steer_ff_bound_mppb;
        let mut c = DisciplinedClock::with_config(cfg);
        let truef = |temp: i64| f0 + k_true * (temp - tref);
        let warm = |n: i64| tref + (10.0 * (n as f64 * core::f64::consts::TAU / 60.0).sin()) as i64;
        for n in 0..1000i64 {
            let t = warm(n);
            c.update_temp(t);
            c.update_freq(1_000_000_000 + truef(t) / 1000);
        }
        // 緩やかなランプ (8 サンプルに 1 raw = 0.125 raw/sample)。lock を保ったまま温度と周波数を更新。
        let base = warm(999);
        let mut checked = 0;
        for h in 1..=200i64 {
            let t = base + h / 8;
            c.update_temp(t);
            c.update_freq(1_000_000_000 + truef(t) / 1000);
            let predicted = c.predicted_freq_mppb();
            let steering = c.steering_freq_mppb();
            // 緩ランプの matched-lead FF 偏差は bound 内 → clamp は dormant。
            assert!(
                (predicted - c.freq_mppb()).abs() <= bound,
                "gentle ramp (temp-ff on) FF should stay within bound: delta={} bound={}",
                predicted - c.freq_mppb(),
                bound
            );
            assert_eq!(steering, predicted, "temp-ff gentle ramp: clamp must stay dormant");
            checked += 1;
        }
        assert!(checked > 0);
    }

    #[test]
    fn corrected_slope_projects_the_frequency_slope() {
        // 凍結状態を直接与えて slope 項の数学を厳密に固定する。
        let mut c = DisciplinedClock::new();
        c.freq_mppb = 0; // level 0 → level 補正を消して slope 項を単離
        c.freq_slope_mppb = 1000; // +1 ppb/s ランプ (mppb/sample)
        let d = 100i64 * 1_000_000_000; // 100 s 経過
        // slope 項 = slope·d²/2e21 = 1000·(1e11)²/2e21 = 1000·1e22/2e21 = 5000 ns。
        assert_eq!(c.corrected(d), d, "level 0 → level 補正なし");
        assert_eq!(c.corrected_slope(d), d - 5000, "slope 項 = 5000 ns");
        // 負の slope は逆符号。
        c.freq_slope_mppb = -1000;
        assert_eq!(c.corrected_slope(d), d + 5000);
    }

    #[test]
    fn slope_aware_holdover_cuts_ramp_endpoint_error() {
        // 温度ランプ下の holdover: level-only は O(H²)、slope-aware は O(H)。
        // 凍結状態を直接与え (tracker 収束を介さず厳密に)、H 秒の holdover 終端誤差を比べる。
        let f0 = 3000i64; // +3000 ppb 始点
        let r = 2i64; // +2 ppb/s ランプ
        let h = 300i64;
        let mut c = DisciplinedClock::new();
        c.freq_mppb = f0 * 1000; // level = 始点 (mppb)
        c.freq_slope_mppb = r * 1000; // slope = ランプ率 (mppb/sample)
        c.update_epoch(0, 0, 0);
        // holdover 中、水晶はランプを継続: 秒 k の周波数 = f0 + r·k (ppb)。
        // local 経過 d = Σ_{k=0}^{H-1}(1e9 + (f0 + r·k)) ns。
        let mut d = 0i64;
        for k in 0..h {
            d += 1_000_000_000 + f0 + r * k;
        }
        let unix_end = h * 1_000_000_000; // 真の経過 = H 秒
        let e_level = c.prediction_residual_ns(d as u64, unix_end).unwrap();
        let e_slope = c.prediction_residual_holdover_ns(d as u64, unix_end).unwrap();
        // level-only は ~r·H²/2 ≈ 90µs、slope-aware は O(H) ~数百 ns。10× 以上の改善を回帰固定。
        assert!(
            e_level.abs() > 10_000,
            "level-only holdover error should be large under a ramp: {e_level}ns"
        );
        assert!(
            e_slope.abs() * 10 < e_level.abs(),
            "slope-aware should cut holdover endpoint error >10x: level={e_level}ns slope={e_slope}ns"
        );
    }

    #[test]
    fn slope_aware_equals_level_when_no_slope() {
        // 定常 (slope=0) では slope-aware = level-only (無害)。
        let mut c = DisciplinedClock::new();
        for _ in 0..40 {
            c.update_freq(1_000_002_500); // +2500 ppb 一定 → slope 0
        }
        assert_eq!(c.freq_slope_mppb(), 0);
        c.update_epoch(0, 0, 0);
        let d = 300i64 * (1_000_000_000 + 2500); // 300 s ぶんの local tick
        let unix = 300i64 * 1_000_000_000;
        assert_eq!(
            c.prediction_residual_ns(d as u64, unix),
            c.prediction_residual_holdover_ns(d as u64, unix),
            "slope 0 では slope-aware は level-only と一致するべき"
        );
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
