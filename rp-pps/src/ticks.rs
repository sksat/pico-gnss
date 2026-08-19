//! One timebase for events that are not one second apart.
//!
//! [`crate::pps_capture_program`] hands back a 32-bit down-counter value at every rising edge, and
//! [`crate::PpsEdgeTimeline`] turns a stream of those into intervals — which is all a 1PPS needs,
//! because the next edge is always a second away and an interval is always positive.
//!
//! A frame arriving on a wire is not like that. It happens when it happens, several may arrive in
//! a burst, and what is wanted is not the gap between two of them but *when* each one was, on a
//! scale shared with everything else being timestamped. So the counter has to be carried past its
//! wrap, and it has to run the other way: a down-counter counts backwards, and a timeline that
//! counts backwards is confusing to reason about at every call site.
//!
//! Both are this module's job. It takes raw counter values and gives back ticks that only ever
//! increase.
//!
//! **It must be fed.** The wrap is 2³² ticks — about 68 s at 125 MHz, where a tick is two cycles —
//! and nothing in a 32-bit value says how many wraps have gone by. Two observations more than half
//! a wrap apart are indistinguishable from two that are half a wrap apart the other way, so a
//! caller that goes quiet for 34 seconds and comes back has lost the thread. Feeding it the 1PPS
//! it already captures is enough, and is what the firmware does.

/// A 32-bit down-counter, carried forward into a 64-bit timeline that counts up.
#[derive(Clone, Copy, Debug, Default)]
pub struct TickTimeline {
    started: bool,
    /// The last raw counter value seen.
    last_raw: u32,
    /// Ticks since the first observation.
    ticks: u64,
    /// Ticks the counter loses each time it captures, added back on every observation.
    ///
    /// A counter cannot decrement while it is pushing. For a 1PPS that is one capture a second and
    /// the loss is beneath notice; for a program that captures a burst it is still one per burst,
    /// but there is no reason to leave it in when it is a known constant.
    toll: u64,
}

impl TickTimeline {
    pub const fn new() -> Self {
        Self::with_toll(0)
    }

    /// A timeline whose counter loses `toll` ticks at every capture — see
    /// [`crate::EVENT_CAPTURE_TOLL_TICKS`].
    pub const fn with_toll(toll: u64) -> Self {
        Self {
            started: false,
            last_raw: 0,
            ticks: 0,
            toll,
        }
    }

    /// Take one raw counter value and place it on the timeline.
    ///
    /// The first observation is the origin and returns zero. Every one after that is the number of
    /// ticks since then, however many wraps have gone by — as long as no two consecutive
    /// observations are more than half a wrap apart.
    pub fn observe(&mut self, raw: u32) -> u64 {
        if !self.started {
            self.started = true;
            self.last_raw = raw;
            return 0;
        }
        // The counter runs down, so time elapsed is how far it fell. Wrapping subtraction is what
        // carries that across the 2³² boundary, and it is why the caller has to come back inside
        // half a wrap: past that, falling a long way and rising a short way look the same.
        self.ticks += self.last_raw.wrapping_sub(raw) as u64 + self.toll;
        self.last_raw = raw;
        self.ticks
    }

    /// Ticks since the origin, without taking a new observation.
    pub fn ticks(&self) -> u64 {
        self.ticks
    }

    /// Whether anything has been observed yet.
    pub fn started(&self) -> bool {
        self.started
    }
}

/// Ticks to nanoseconds at a given system clock.
///
/// Multiplies before dividing, in `u128`, so there is no per-tick truncation to accumulate — exact
/// at 125 MHz, where a tick is 16 ns.
pub fn ticks_to_ns(ticks: u64, clk_hz: u32) -> u64 {
    (ticks as u128 * crate::CAPTURE_CYCLES_PER_TICK as u128 * 1_000_000_000 / clk_hz as u128) as u64
}

/// Nanoseconds to ticks at a given system clock.
pub fn ns_to_ticks(ns: u64, clk_hz: u32) -> u64 {
    (ns as u128 * clk_hz as u128 / (crate::CAPTURE_CYCLES_PER_TICK as u128 * 1_000_000_000)) as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    const CLK: u32 = 125_000_000;
    /// Ticks in one second at 125 MHz with two cycles per tick.
    const PER_SECOND: u32 = CLK / crate::CAPTURE_CYCLES_PER_TICK;

    #[test]
    fn the_first_observation_is_the_origin() {
        let mut t = TickTimeline::new();
        assert!(!t.started());
        assert_eq!(t.observe(0xDEAD_BEEF), 0);
        assert!(t.started());
    }

    #[test]
    fn a_counter_that_runs_down_gives_a_timeline_that_runs_up() {
        let mut t = TickTimeline::new();
        t.observe(1_000_000);
        assert_eq!(t.observe(999_000), 1_000);
        assert_eq!(t.observe(998_500), 1_500);
    }


    #[test]
    fn a_counter_that_stops_to_capture_has_the_difference_added_back() {
        // Two ticks per capture, and one capture between each pair of observations.
        let mut t = TickTimeline::with_toll(2);
        t.observe(1_000_000);
        assert_eq!(t.observe(999_000), 1_002);
        assert_eq!(t.observe(998_000), 2_004);
    }

    #[test]
    fn a_toll_of_zero_is_the_plain_timeline() {
        let mut a = TickTimeline::new();
        let mut b = TickTimeline::with_toll(0);
        a.observe(500);
        b.observe(500);
        assert_eq!(a.observe(400), b.observe(400));
    }

    #[test]
    fn the_wrap_is_carried() {
        let mut t = TickTimeline::new();
        t.observe(100);
        // Falling past zero: 100 -> 0 is 100 ticks, and the rest is on the other side.
        assert_eq!(t.observe(u32::MAX - 899), 1_000);
        assert_eq!(t.observe(u32::MAX - 1_899), 2_000);
    }

    #[test]
    fn a_second_of_pulses_lands_a_second_later() {
        let mut t = TickTimeline::new();
        let mut raw: u32 = 0x1234_5678;
        t.observe(raw);
        for second in 1..=120u64 {
            raw = raw.wrapping_sub(PER_SECOND);
            let ticks = t.observe(raw);
            let ns = ticks_to_ns(ticks, CLK);
            assert_eq!(ns, second * 1_000_000_000, "second {second}");
        }
    }

    #[test]
    fn ticks_and_nanoseconds_round_trip_on_the_tick() {
        // Only on multiples of the tick, which at 125 MHz is 16 ns. A duration that falls between
        // two ticks cannot come back unchanged, and pretending otherwise would be a test that
        // asserts the counter has a resolution it does not have.
        for ns in [0u64, 16, 992, 1_000_000, 1_000_000_000] {
            assert_eq!(ns % 16, 0, "{ns} is not a whole number of ticks");
            let ticks = ns_to_ticks(ns, CLK);
            assert_eq!(ticks_to_ns(ticks, CLK), ns, "{ns} ns");
        }
    }

    #[test]
    fn a_duration_between_two_ticks_is_truncated_not_rounded() {
        // 1000 ns is 62.5 ticks. What comes back is 62 of them.
        assert_eq!(ns_to_ticks(1_000, CLK), 62);
        assert_eq!(ticks_to_ns(62, CLK), 992);
    }

    #[test]
    fn events_a_few_microseconds_apart_are_resolved() {
        // What the link needs: a frame leaving and the far side's frame arriving are hundreds of
        // microseconds apart, and the tick is 16 ns.
        let mut t = TickTimeline::new();
        let raw: u32 = 0x8000_0000;
        t.observe(raw);
        let ticks = t.observe(raw.wrapping_sub(ns_to_ticks(102_400, CLK) as u32));
        assert_eq!(ticks_to_ns(ticks, CLK), 102_400);
    }
}
