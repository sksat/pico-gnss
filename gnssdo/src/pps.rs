//! Tracking of the PPS (pulse-per-second) edge stream.
//!
//! A GNSS PPS is ideally exactly 1 Hz. From the sequence of received rising-edge timestamps,
//! this classifies lock (~1 s), missed pulses, and glitches/jitter. It is app-specific logic
//! that no library provides, so it lives here and is host-tested (`cargo test -p gnssdo`).
//!
//! HAL-agnostic: a timestamp is just a microsecond `u64`. On the firmware side, pass
//! `embassy_time::Instant::now().as_micros()`.

use core::num::NonZeroU64;

/// Nominal PPS interval (1 second = 1_000_000 us). Default for a 1 Hz PPS.
pub const NOMINAL_US: u64 = 1_000_000;

/// Default lock tolerance (±50 ms). Beyond this is treated as a missed pulse or a glitch.
pub const TOLERANCE_US: u64 = 50_000;

/// Configuration for [`PpsTracker`]. Made configurable to support non-1 Hz PPS (e.g. 10 Hz) and
/// receiver/capture-specific tolerances. `Default` is 1 Hz / ±50 ms.
///
/// **Note**: setting `nominal_us` to a non-1 Hz value only affects [`PpsTracker`]'s own lock/miss
/// classification. The GPSDO frequency discipline ([`crate::DisciplinedClock::update_freq`]) is
/// currently **1 Hz only** (it evaluates the interval against 1e9 ns), so feeding a non-1 Hz
/// `Locked` straight into frequency discipline does not work.
///
/// (`Debug`/`Default` is the minimum required by [`PpsTracker`]'s derive.)
#[derive(Debug)]
pub struct PpsTrackerConfig {
    /// Nominal PPS interval (µs). 1_000_000 for 1 Hz. `NonZeroU64` rules out 0 (division by zero).
    pub nominal_us: NonZeroU64,
    /// Lock tolerance (µs). Within nominal ± this is Locked; widened by the multiple on misses.
    /// 0 is valid too (strict: only an exact nominal locks), so it is not `NonZero`.
    pub tolerance_us: u64,
}

impl PpsTrackerConfig {
    /// Defaults (1 Hz, ±50 ms). `const` so the `const fn` [`PpsTracker::new`] can use it.
    pub const DEFAULT: Self = Self {
        nominal_us: NonZeroU64::new(NOMINAL_US).unwrap(),
        tolerance_us: TOLERANCE_US,
    };
}

impl Default for PpsTrackerConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Result of recording one edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PpsEvent {
    /// First edge (interval not yet known).
    First,
    /// Nominal 1 s ± tolerance. Stable lock.
    Locked { interval_us: u64 },
    /// Off-nominal. `missed` is the estimated number of dropped pulses (0 for a glitch/jitter).
    Irregular { interval_us: u64, missed: u32 },
    /// The input timestamp went backwards (timer wrap / out-of-order / reset). `backwards_us` is
    /// how far back. No interval can be measured, so it is not used for discipline; internal state
    /// rebases to the current value and resumes from the next edge.
    NonMonotonic { backwards_us: u64 },
}

/// State machine over the PPS edge stream.
#[derive(Debug, Default)]
pub struct PpsTracker {
    config: PpsTrackerConfig,
    last_us: Option<u64>,
    count: u32,
}

impl PpsTracker {
    /// Create with the defaults (1 Hz, ±50 ms).
    pub const fn new() -> Self {
        Self::with_config(PpsTrackerConfig::DEFAULT)
    }

    /// Create with a given configuration. `const fn` so it can be used in `static` init.
    pub const fn with_config(config: PpsTrackerConfig) -> Self {
        Self {
            config,
            last_us: None,
            count: 0,
        }
    }

    /// Current configuration.
    pub fn config(&self) -> &PpsTrackerConfig {
        &self.config
    }

    /// Total number of edges recorded so far.
    pub fn count(&self) -> u32 {
        self.count
    }

    /// Record one rising edge and return the classification.
    /// `now_us` is a monotonically increasing microsecond timestamp.
    pub fn record(&mut self, now_us: u64) -> PpsEvent {
        self.count += 1;
        let nominal = self.config.nominal_us.get();
        let event = match self.last_us {
            None => PpsEvent::First,
            // Backwards (timer wrap / out-of-order / reset): no interval; report it instead of
            // silently producing 0.
            Some(prev) if now_us < prev => PpsEvent::NonMonotonic {
                backwards_us: prev - now_us,
            },
            Some(prev) => {
                let interval = now_us - prev; // now_us >= prev guaranteed by the guard above
                // Nearest integer multiple of the nominal interval (rounded).
                let n = (interval + nominal / 2) / nominal;
                if n >= 1 {
                    let expected = n * nominal;
                    let error = interval.abs_diff(expected);
                    // Widen the tolerance by the multiple as well.
                    if error <= self.config.tolerance_us * n {
                        if n == 1 {
                            PpsEvent::Locked {
                                interval_us: interval,
                            }
                        } else {
                            PpsEvent::Irregular {
                                interval_us: interval,
                                missed: (n - 1) as u32,
                            }
                        }
                    } else {
                        PpsEvent::Irregular {
                            interval_us: interval,
                            missed: 0,
                        }
                    }
                } else {
                    // Under 0.5 s = glitch.
                    PpsEvent::Irregular {
                        interval_us: interval,
                        missed: 0,
                    }
                }
            }
        };
        self.last_us = Some(now_us);
        event
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::num::NonZeroU64;

    #[test]
    fn non_monotonic_input_is_reported_and_rebases() {
        let mut t = PpsTracker::new();
        t.record(2_000_000);
        // Earlier than the previous edge → NonMonotonic (500 ms back), not a silent 0.
        assert_eq!(
            t.record(1_500_000),
            PpsEvent::NonMonotonic {
                backwards_us: 500_000
            }
        );
        // Already rebased to the current value, so the next +1 s locks again as usual.
        assert_eq!(
            t.record(2_500_000),
            PpsEvent::Locked {
                interval_us: 1_000_000
            }
        );
        assert_eq!(t.count(), 3);
    }

    #[test]
    fn custom_tolerance_tightens_lock_window() {
        // With ±5 ms tolerance, a +30 ms jitter that was Locked at the default (±50 ms) is Irregular.
        let cfg = PpsTrackerConfig {
            tolerance_us: 5_000,
            ..PpsTrackerConfig::DEFAULT
        };
        let mut t = PpsTracker::with_config(cfg);
        t.record(0);
        assert_eq!(
            t.record(1_030_000),
            PpsEvent::Irregular {
                interval_us: 1_030_000,
                missed: 0
            }
        );
    }

    #[test]
    fn custom_nominal_supports_non_1hz() {
        // With a 100 ms nominal (10 Hz), one period still Locks. Confirms it is not 1 Hz-only.
        let cfg = PpsTrackerConfig {
            nominal_us: NonZeroU64::new(100_000).unwrap(),
            ..PpsTrackerConfig::DEFAULT
        };
        let mut t = PpsTracker::with_config(cfg);
        t.record(0);
        assert_eq!(
            t.record(100_000),
            PpsEvent::Locked {
                interval_us: 100_000
            }
        );
    }

    #[test]
    fn first_edge_is_first() {
        let mut t = PpsTracker::new();
        assert_eq!(t.record(5_000_000), PpsEvent::First);
        assert_eq!(t.count(), 1);
    }

    #[test]
    fn nominal_one_second_is_locked() {
        let mut t = PpsTracker::new();
        t.record(0);
        assert_eq!(
            t.record(NOMINAL_US),
            PpsEvent::Locked {
                interval_us: NOMINAL_US
            }
        );
        assert_eq!(t.count(), 2);
    }

    #[test]
    fn small_jitter_within_tolerance_is_locked() {
        let mut t = PpsTracker::new();
        t.record(0);
        // +30 ms jitter is within tolerance.
        assert_eq!(
            t.record(1_030_000),
            PpsEvent::Locked {
                interval_us: 1_030_000
            }
        );
    }

    #[test]
    fn one_missed_pulse() {
        let mut t = PpsTracker::new();
        t.record(0);
        // ~2 s gap = 1 dropped pulse.
        assert_eq!(
            t.record(2_000_000),
            PpsEvent::Irregular {
                interval_us: 2_000_000,
                missed: 1
            }
        );
    }

    #[test]
    fn three_missed_pulses() {
        let mut t = PpsTracker::new();
        t.record(0);
        assert_eq!(
            t.record(4_010_000),
            PpsEvent::Irregular {
                interval_us: 4_010_000,
                missed: 3
            }
        );
    }

    #[test]
    fn too_short_is_glitch() {
        let mut t = PpsTracker::new();
        t.record(0);
        // 0.2 s = glitch (not a dropped pulse).
        assert_eq!(
            t.record(200_000),
            PpsEvent::Irregular {
                interval_us: 200_000,
                missed: 0
            }
        );
    }

    #[test]
    fn off_nominal_between_multiples_is_glitch() {
        let mut t = PpsTracker::new();
        t.record(0);
        // 1.5 s is far from both 1x and 2x = jitter (missed=0).
        assert_eq!(
            t.record(1_500_000),
            PpsEvent::Irregular {
                interval_us: 1_500_000,
                missed: 0
            }
        );
    }

    #[test]
    fn lock_reacquires_after_gap() {
        let mut t = PpsTracker::new();
        t.record(0);
        t.record(2_000_000); // missed 1
        // If the next one is 1 s again, it returns to lock.
        assert_eq!(
            t.record(3_000_000),
            PpsEvent::Locked {
                interval_us: 1_000_000
            }
        );
        assert_eq!(t.count(), 3);
    }
}
