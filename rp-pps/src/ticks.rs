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
    /// The state machine's capture tally at the last observation, so the tolls paid by captures
    /// that were drained rather than read can still be charged.
    last_captures: u64,
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
            last_captures: 0,
            toll,
        }
    }

    /// A timeline whose origin is the moment the counter was enabled, not its first capture.
    ///
    /// [`with_toll`](Self::with_toll) makes the first observation the origin, which is right when
    /// the only thing wanted is intervals. It is wrong when the ticks have to mean the same as
    /// another counter's: state machines started by one [`crate::embassy::start_in_sync`] write all
    /// hold zero at that instant, so a timeline anchored there is on the same scale as
    /// [`crate::PpsEdgeTimeline::from_counter_start`] — and that is the scale a capture has to be on
    /// for `now_from_capture_ns` to turn it into UTC.
    pub const fn from_counter_start_with_toll(toll: u64) -> Self {
        Self {
            // Zero is not "nothing seen yet" here, it is the value the counter held when the write
            // enabled it. The first capture is an interval from that, like every one after it.
            started: true,
            last_raw: 0,
            ticks: 0,
            last_captures: 0,
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

    /// Take one raw value, knowing how many times the counter has stopped to capture.
    ///
    /// [`observe`](Self::observe) charges one toll per call, which is right only when the caller
    /// reads every capture. A caller that reads one value and drains the rest — which is what
    /// watching a *frame* means, since every edge in it stops the counter — has skipped tolls that
    /// were nonetheless paid, and the timeline falls behind by that much every frame. It does not
    /// look like error: it looks like a clock running slow.
    ///
    /// `captures` is the state machine's own tally at the moment `raw` was pushed. What is charged
    /// is the number of stops since the last observation, however many of them were read.
    pub fn observe_with_captures(&mut self, raw: u32, captures: u64) -> u64 {
        if !self.started {
            self.started = true;
            self.last_raw = raw;
            self.last_captures = captures;
            return 0;
        }
        let stops = captures.saturating_sub(self.last_captures);
        self.ticks += self.last_raw.wrapping_sub(raw) as u64 + self.toll * stops;
        self.last_raw = raw;
        self.last_captures = captures;
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

/// Ticks between two events timestamped by *different* counters in the same started set.
///
/// The counters were started together and are equal — that much was measured on the board. What
/// they are not is identical over time: each one stops for [`crate::EVENT_CAPTURE_TOLL_TICKS`]
/// every time *it* captures, so two counters watching different pins fall behind each other in
/// proportion to how often each one fires. On a link where one pin carries a frame a second and
/// the other carries two, that is thirty-two nanoseconds a second of divergence.
///
/// So the correction is not a constant, it is a count. `captures_before` is how many times each
/// counter had already captured when it took the value it is being asked about; the value itself
/// is pushed before its own toll and does not need to include it.
///
/// Returns the ticks from `earlier` to `later`. Both are raw down-counter values.
pub fn ticks_between(
    earlier: u32,
    earlier_captures: u64,
    later: u32,
    later_captures: u64,
    toll: u64,
) -> i64 {
    let counted = earlier.wrapping_sub(later) as i64;
    counted - toll as i64 * (earlier_captures as i64 - later_captures as i64)
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
    fn two_counters_that_have_captured_equally_need_no_correction() {
        // 1000 ticks apart, and each has stopped the same number of times.
        assert_eq!(ticks_between(10_000, 7, 9_000, 7, 2), 1_000);
    }

    #[test]
    fn a_counter_that_has_stopped_more_often_reads_high() {
        // The earlier counter has captured five more times than the later one, so it is ten ticks
        // behind - it reads ten higher than it would have, and the gap looks ten too long.
        assert_eq!(ticks_between(10_000, 12, 9_000, 7, 2), 990);
        // And the other way round.
        assert_eq!(ticks_between(10_000, 7, 9_000, 12, 2), 1_010);
    }

    #[test]
    fn the_gap_is_carried_across_the_wrap() {
        assert_eq!(ticks_between(100, 3, u32::MAX - 899, 3, 2), 1_000);
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

    #[test]
    fn a_timeline_anchored_at_the_counter_start_measures_from_the_start() {
        // Started by one `start_in_sync` write, the counter holds zero at that instant. The very
        // first capture is therefore already an interval — 0.5 ms after the start, here — and not
        // an origin that reads zero.
        let mut t = TickTimeline::from_counter_start_with_toll(0);
        let elapsed = ns_to_ticks(500_000, CLK);
        assert_eq!(t.observe(0u32.wrapping_sub(elapsed as u32)), elapsed);
    }

    #[test]
    fn a_frame_and_a_pps_on_the_same_start_agree_on_when_they_were() {
        // The point of anchoring at the start: a frame's counter value has to mean the same thing
        // as the 1PPS counter's, because it is `now_from_capture_ns` — fed from the 1PPS timeline —
        // that turns it into UTC. Two counters, same program, same start, same instant.
        let at_ns = 3_000_000_000; // 3 s in, well past one 68 s wrap of neither
        let raw = 0u32.wrapping_sub(ns_to_ticks(at_ns, CLK) as u32);

        let mut pps = crate::PpsEdgeTimeline::from_counter_start(CLK);
        let mut frame = TickTimeline::from_counter_start_with_toll(0);

        assert_eq!(pps.observe(raw).edge_ns, at_ns);
        assert_eq!(ticks_to_ns(frame.observe(raw), CLK), at_ns);
    }

    #[test]
    fn tolls_paid_by_captures_that_were_drained_are_still_charged() {
        // A frame stops the counter once per edge in it, and the caller reads the first and drains
        // the rest. Charging one toll per read loses the others, every frame, for as long as the
        // link is up — which does not read as error, it reads as a clock running slow.
        let mut t = TickTimeline::from_counter_start_with_toll(2);
        let elapsed = ns_to_ticks(1_000_000, CLK);
        // Eight captures went by; one of them is the value being read.
        assert_eq!(
            t.observe_with_captures(0u32.wrapping_sub(elapsed as u32), 8),
            elapsed + 2 * 8
        );
    }

    #[test]
    fn only_the_stops_since_the_last_reading_are_charged() {
        let mut t = TickTimeline::from_counter_start_with_toll(2);
        let first = ns_to_ticks(1_000_000, CLK);
        t.observe_with_captures(0u32.wrapping_sub(first as u32), 4);
        let second = ns_to_ticks(2_000_000, CLK);
        assert_eq!(
            t.observe_with_captures(0u32.wrapping_sub(second as u32), 7),
            second + 2 * 7
        );
    }

    #[test]
    fn a_capture_tally_that_goes_backwards_charges_nothing_rather_than_wrapping() {
        // The tally is the state machine's, read across a lock. It should only ever grow, but a
        // wrapping subtraction on a `u64` that did go backwards would add several thousand years.
        let mut t = TickTimeline::from_counter_start_with_toll(2);
        let elapsed = ns_to_ticks(1_000_000, CLK);
        t.observe_with_captures(0u32.wrapping_sub(elapsed as u32), 9);
        let later = ns_to_ticks(2_000_000, CLK);
        assert_eq!(
            t.observe_with_captures(0u32.wrapping_sub(later as u32), 3),
            later + 2 * 9
        );
    }

    #[test]
    fn the_toll_is_added_from_the_first_capture_of_an_anchored_timeline() {
        // A capturing program cannot decrement while it pushes, so every capture loses `toll`
        // ticks — including the first one after the start, which an unanchored timeline never
        // charges because it treats that capture as the origin.
        let mut t = TickTimeline::from_counter_start_with_toll(2);
        let elapsed = ns_to_ticks(1_000_000, CLK);
        assert_eq!(t.observe(0u32.wrapping_sub(elapsed as u32)), elapsed + 2);
    }
}
