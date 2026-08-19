//! Holding a local clock to what NTP says the time is.
//!
//! A client that only ever applied the latest offset would carry the whole of that measurement's
//! noise into its clock, and would have nothing to say between measurements. This keeps two numbers
//! instead: how far the local clock is from UTC, and how fast it is drifting away. The second one
//! is what lets the clock be read at any moment, and what a 1PPS output has to be steered by.
//!
//! It is an alpha-beta tracker: predict the offset from the previous estimate, and split the
//! residual between the two states. Integer-only, and the arithmetic stays in nanoseconds and parts
//! per billion, which is what the layer below wants.
//!
//! What it does not do is decide the delay. The offset it is handed has already had the path taken
//! out of it, by [`crate::client::measure`] or by [`crate::client::accept_broadcast`].

/// How hard to pull, and when to give up and jump.
#[derive(Clone, Copy, Debug)]
pub struct DisciplineConfig {
    /// A residual this large (ns) is not a drifting clock, so the estimate restarts on it rather
    /// than steering towards it over minutes.
    pub step_threshold_ns: i64,
    /// Phase gain, as a reciprocal: the residual is divided by this before it moves the offset.
    pub phase_gain_inv: i64,
    /// Frequency gain, as a reciprocal. Larger than the phase gain, because rate is the slower
    /// state and taking it straight from one residual would make the loop ring.
    pub freq_gain_inv: i64,
    /// Measurements to take before the estimate is worth using.
    pub lock_after: u32,
}

impl Default for DisciplineConfig {
    /// Gains for a measurement every second or so, from a source whose noise is microseconds.
    ///
    /// The phase gain settles a step in about a minute; the frequency gain is eight times weaker,
    /// which keeps a single noisy measurement from being read as a rate change.
    fn default() -> Self {
        Self {
            step_threshold_ns: 10_000_000,
            phase_gain_inv: 8,
            freq_gain_inv: 64,
            lock_after: 16,
        }
    }
}

/// What one measurement did to the estimate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DisciplineUpdate {
    /// The estimate was restarted on this measurement rather than steered by it.
    pub stepped: bool,
    /// How far the measurement was from what the estimate predicted (ns). This is the loop's error
    /// signal, and with a settled clock it is the measurement noise.
    pub residual_ns: i64,
    /// Offset from local time to UTC at this measurement (ns).
    pub offset_ns: i64,
    /// How fast the local clock is running against UTC (ppb, positive when it gains).
    pub drift_ppb: i64,
}

/// The estimate: an offset, a rate, and the local time both were true at.
#[derive(Clone, Copy, Debug)]
pub struct NtpDiscipline {
    cfg: DisciplineConfig,
    started: bool,
    t_ref_ns: i64,
    offset_ns: i64,
    drift_ppb: i64,
    updates: u32,
}

impl NtpDiscipline {
    pub fn new(cfg: DisciplineConfig) -> Self {
        Self {
            cfg,
            started: false,
            t_ref_ns: 0,
            offset_ns: 0,
            drift_ppb: 0,
            updates: 0,
        }
    }

    /// UTC for a local timestamp, or `None` before the first measurement.
    ///
    /// Valid between measurements as well as at them: that is what the rate state is for.
    pub fn utc_at(&self, local_ns: i64) -> Option<i64> {
        if !self.started {
            return None;
        }
        Some(local_ns + self.predict(local_ns))
    }

    /// How fast the local clock runs against UTC (ppb, positive when it gains).
    pub fn drift_ppb(&self) -> i64 {
        self.drift_ppb
    }

    /// Offset at the last measurement (ns).
    pub fn offset_ns(&self) -> i64 {
        self.offset_ns
    }

    /// Whether enough measurements have gone in for the estimate to be worth acting on.
    pub fn locked(&self) -> bool {
        self.updates >= self.cfg.lock_after
    }

    /// Take one measurement: `offset_ns` is UTC minus local at local time `local_ns`.
    pub fn observe(&mut self, local_ns: i64, offset_ns: i64) -> DisciplineUpdate {
        let dt_ns = local_ns.wrapping_sub(self.t_ref_ns);
        let residual = if self.started {
            offset_ns - self.predict(local_ns)
        } else {
            0
        };

        // A gap with no measurements leaves nothing to divide by, and a residual out of range is
        // not a clock that drifted there.
        let step = !self.started || dt_ns <= 0 || residual.abs() > self.cfg.step_threshold_ns;
        if step {
            self.offset_ns = offset_ns;
            self.t_ref_ns = local_ns;
            self.started = true;
            self.updates = 0;
        } else {
            self.offset_ns = offset_ns - residual + residual / self.cfg.phase_gain_inv;
            // The residual is a phase error; over `dt` it is a rate error of residual/dt. A
            // positive residual means the offset fell less than predicted, so the clock is gaining
            // less than the estimate says.
            let rate_ppb = (residual as i128) * 1_000_000_000 / dt_ns as i128;
            self.drift_ppb -= (rate_ppb / self.cfg.freq_gain_inv as i128) as i64;
            self.t_ref_ns = local_ns;
            self.updates = self.updates.saturating_add(1);
        }

        DisciplineUpdate {
            stepped: step,
            residual_ns: residual,
            offset_ns: self.offset_ns,
            drift_ppb: self.drift_ppb,
        }
    }

    /// Offset the estimate expects at `local_ns`.
    ///
    /// A clock that gains loses offset: UTC minus local shrinks by `drift_ppb` nanoseconds every
    /// second.
    fn predict(&self, local_ns: i64) -> i64 {
        let dt_ns = local_ns.wrapping_sub(self.t_ref_ns) as i128;
        let slip = self.drift_ppb as i128 * dt_ns / 1_000_000_000;
        (self.offset_ns as i128 - slip) as i64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A local clock that gains `ppb` against UTC, sampled once a second.
    struct Fake {
        ppb: i64,
        utc_start_ns: i64,
    }

    impl Fake {
        /// Local reading after `real_ns` of real time. A clock that gains reads further ahead than
        /// the time that has actually passed, so a local interval is not a real one.
        fn local_at(&self, real_ns: i64) -> i64 {
            real_ns + real_ns * self.ppb / 1_000_000_000
        }

        /// True UTC after `real_ns` of real time.
        fn utc_at(&self, real_ns: i64) -> i64 {
            self.utc_start_ns + real_ns
        }

        /// Local reading at `second` seconds of real time.
        fn local_ns(&self, second: i64) -> i64 {
            self.local_at(second * 1_000_000_000)
        }

        /// True offset at that moment: UTC minus local.
        fn offset_ns(&self, second: i64) -> i64 {
            self.utc_at(second * 1_000_000_000) - self.local_ns(second)
        }
    }

    /// Repeatable jitter in [-span, span].
    fn jitter(seed: &mut u64, span: i64) -> i64 {
        *seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        (((*seed >> 33) as i64) % (2 * span + 1)) - span
    }

    #[test]
    fn nothing_can_be_read_before_the_first_measurement() {
        let d = NtpDiscipline::new(DisciplineConfig::default());
        assert_eq!(d.utc_at(0), None);
        assert!(!d.locked());
    }

    #[test]
    fn the_first_measurement_is_taken_whole() {
        let mut d = NtpDiscipline::new(DisciplineConfig::default());
        let update = d.observe(1_000, 42_000);
        assert!(update.stepped);
        assert_eq!(d.utc_at(1_000), Some(43_000));
    }

    #[test]
    fn a_constant_rate_error_is_learned() {
        // 20 ppm is a plausible crystal. Noise-free, so what is left at the end is the loop.
        let clock = Fake {
            ppb: 20_000,
            utc_start_ns: 1_700_000_000_000_000_000,
        };
        let mut d = NtpDiscipline::new(DisciplineConfig::default());
        for second in 0..600 {
            d.observe(clock.local_ns(second), clock.offset_ns(second));
        }
        assert!(d.locked());
        assert!(
            (d.drift_ppb() - 20_000).abs() < 50,
            "learned {} ppb",
            d.drift_ppb()
        );
        let second = 600;
        let error =
            d.utc_at(clock.local_ns(second)).unwrap() - clock.utc_at(second * 1_000_000_000);
        assert!(error.abs() < 200, "clock is off by {error} ns");
    }

    #[test]
    fn a_rate_the_estimate_never_saw_is_still_read_between_measurements() {
        // The point of holding a rate: ask for a time no measurement was taken at.
        let clock = Fake {
            ppb: 20_000,
            utc_start_ns: 1_700_000_000_000_000_000,
        };
        let mut d = NtpDiscipline::new(DisciplineConfig::default());
        for second in 0..600 {
            d.observe(clock.local_ns(second), clock.offset_ns(second));
        }
        // Half a second of real time past the last measurement.
        let real = 600 * 1_000_000_000 + 500_000_000;
        let utc = d.utc_at(clock.local_at(real)).unwrap();
        let truth = clock.utc_at(real);
        assert!((utc - truth).abs() < 200, "off by {} ns", utc - truth);
    }

    #[test]
    fn measurement_noise_averages_out() {
        let clock = Fake {
            ppb: -12_000,
            utc_start_ns: 1_700_000_000_000_000_000,
        };
        let mut d = NtpDiscipline::new(DisciplineConfig::default());
        let mut seed = 0x1234_5678_9ABC_DEF0u64;
        for second in 0..900 {
            let noise = jitter(&mut seed, 5_000);
            d.observe(clock.local_ns(second), clock.offset_ns(second) + noise);
        }
        // The measurements were +-5 us; what is left has to be well inside that.
        let second = 900;
        let error =
            d.utc_at(clock.local_ns(second)).unwrap() - clock.utc_at(second * 1_000_000_000);
        assert!(error.abs() < 2_000, "clock is off by {error} ns");
        assert!(
            (d.drift_ppb() + 12_000).abs() < 500,
            "learned {} ppb",
            d.drift_ppb()
        );
    }

    #[test]
    fn a_jump_is_stepped_rather_than_steered_towards() {
        let clock = Fake {
            ppb: 5_000,
            utc_start_ns: 1_700_000_000_000_000_000,
        };
        let mut d = NtpDiscipline::new(DisciplineConfig::default());
        for second in 0..100 {
            d.observe(clock.local_ns(second), clock.offset_ns(second));
        }
        // The server's idea of the time moves by a second.
        let jumped = clock.offset_ns(100) + 1_000_000_000;
        let update = d.observe(clock.local_ns(100), jumped);
        assert!(update.stepped);
        assert!(!d.locked(), "a step is not a settled estimate");
        assert_eq!(d.utc_at(clock.local_ns(100)), Some(clock.local_ns(100) + jumped));
    }

    #[test]
    fn a_gap_with_no_measurements_does_not_divide_by_it() {
        let mut d = NtpDiscipline::new(DisciplineConfig::default());
        d.observe(1_000_000_000, 0);
        // Same instant twice: the rate term has nothing to work with.
        let update = d.observe(1_000_000_000, 0);
        assert!(update.stepped);
        assert_eq!(d.drift_ppb(), 0);
    }
}
