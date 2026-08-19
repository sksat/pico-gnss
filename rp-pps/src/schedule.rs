//! Placing the output's rising edges on the UTC second without watching them.
//!
//! The usual way to know where a 1PPS edge landed is to capture it — loop the pin back and time it
//! on another state machine. That measures the pad, which is the honest thing to do, and it costs a
//! wire and a state machine.
//!
//! It is not the only way. The output program's edges are separated by exactly the period words it
//! was handed ([`crate::pps_output_program`]): edge to edge is `word + high + OVERHEAD` cycles of a
//! clock the caller already knows. So a scheduler that hands out the words also knows, to the cycle,
//! where every edge after the first one is. Only the first is unknown, and it is one instruction
//! after the state machine was enabled.
//!
//! What that buys is an edge position with no per-edge measurement noise in it. What it costs is
//! that the one unknown never goes away: whatever the local timestamp of `set_enable` was wrong by
//! is a fixed offset on the output forever. That is a constant, and constants are what an
//! oscilloscope is for — but it does mean this cannot be checked from the inside.

use crate::{OUTPUT_OVERHEAD_CYCLES, OutputPeriodDither};

/// How hard to pull an edge back onto the second.
#[derive(Clone, Copy, Debug)]
pub struct PpsScheduleConfig {
    /// Phase gain, as a reciprocal: an edge this late has that fraction of it taken out of the next
    /// period. Correcting the whole error at once would put the loop's own noise on the output.
    pub phase_gain_inv: i64,
    /// The most one period may be stretched or squeezed for phase (ns). A large correction is a
    /// large frequency excursion, and this is a 1PPS output.
    pub max_correction_ns: i64,
}

impl Default for PpsScheduleConfig {
    fn default() -> Self {
        Self {
            phase_gain_inv: 4,
            max_correction_ns: 1_000_000,
        }
    }
}

/// The next edge, and the word that will put it there.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PpsStep {
    /// The word to push to the output state machine.
    pub period_word: u32,
    /// Local time the edge this word positions will happen at (ns).
    pub edge_ns: i64,
    /// The phase correction asked for, after the clamp (ns). What the edge actually moved by is
    /// this to within a cycle; the remainder is carried to the next edge rather than dropped.
    pub correction_ns: i64,
}

/// Tracks where the output's edges are, from the words it hands out.
#[derive(Debug)]
pub struct PpsSchedule {
    clk_hz: u32,
    high_cycles: u32,
    cfg: PpsScheduleConfig,
    dither: OutputPeriodDither,
    edge_ns: i64,
    /// Phase correction asked for but not yet spent, in ns times the clock (so, cycles scaled by
    /// 1e9). A period is a whole number of cycles, and at 125 MHz one cycle is 8 ns; without this
    /// every correction under 8 ns rounds to nothing and the loop stalls with the edge that far
    /// out. Carried, they add up until a cycle is worth taking.
    phase_acc: i64,
}

impl PpsSchedule {
    /// Start from the last edge whose position is already fixed.
    ///
    /// After [`crate::embassy::PpsOutput::new`] that is the *second* edge: the initial period word
    /// pushed there governs the interval from the first edge to it. Use [`first_edge_ns`] and
    /// [`Self::edge_after`] to work it out.
    pub fn new(clk_hz: u32, high_cycles: u32, cfg: PpsScheduleConfig, edge_ns: i64) -> Self {
        Self {
            clk_hz,
            high_cycles,
            cfg,
            dither: OutputPeriodDither::default(),
            edge_ns,
            phase_acc: 0,
        }
    }

    /// Start from the moment the output state machine was enabled.
    ///
    /// The first edge is [`first_edge_ns`] after that, and the initial period word pushed by
    /// [`crate::embassy::PpsOutput::new`] governs the interval to the second edge — which is the
    /// last one already fixed, and so where this begins.
    pub fn at_enable(
        clk_hz: u32,
        high_cycles: u32,
        cfg: PpsScheduleConfig,
        enable_ns: i64,
        initial_period: u32,
    ) -> Self {
        let mut s = Self::new(clk_hz, high_cycles, cfg, first_edge_ns(clk_hz, enable_ns));
        s.edge_ns = s.edge_after(s.edge_ns, initial_period);
        s
    }

    /// Put the next edge on a second boundary in one step, instead of walking it there.
    ///
    /// Steering moves an edge by at most [`PpsScheduleConfig::max_correction_ns`] per second, so an
    /// edge that starts half a second out would take minutes to arrive. At acquisition there is
    /// nothing to protect — no client is listening to an output that is half a second wrong — so it
    /// goes in one period.
    ///
    /// The edge is always *delayed*, never advanced: a period is a count and cannot be negative,
    /// and delaying by `1 s - lateness` reaches the same place as advancing by `lateness`, one
    /// second later. It marks a second either way.
    pub fn acquire(&mut self, freq_mppb: i64, lateness_ns: i64) -> PpsStep {
        let delay = (-lateness_ns).rem_euclid(1_000_000_000);
        let base = self
            .dither
            .next_period(self.clk_hz, freq_mppb, 0, self.high_cycles);
        // Straight to cycles: this one is not a correction to be carried, it is a placement.
        let delay_cycles = (delay as i128 * self.clk_hz as i128 / 1_000_000_000) as i64;
        self.phase_acc = 0;
        let period_word = (base as i64 + delay_cycles) as u32;
        self.edge_ns = self.edge_after(self.edge_ns, period_word);
        PpsStep {
            period_word,
            edge_ns: self.edge_ns,
            correction_ns: -delay,
        }
    }

    /// Local time of the last edge this schedule has committed to (ns).
    pub fn edge_ns(&self) -> i64 {
        self.edge_ns
    }

    /// Where the next edge lands if the next period is the nominal second.
    ///
    /// This is what the caller asks the clock about, before there is a word to be exact with. The
    /// difference between this and where the edge really goes is the correction itself, which is
    /// small and already the thing being driven to zero.
    pub fn predicted_edge_ns(&self) -> i64 {
        self.edge_ns + 1_000_000_000
    }

    /// Where an edge lands one period word later.
    pub fn edge_after(&self, edge_ns: i64, period_word: u32) -> i64 {
        edge_ns + self.cycles_to_ns(period_word as u64 + self.high_cycles as u64 + OUTPUT_OVERHEAD_CYCLES as u64)
    }

    /// Hand out the next period word.
    ///
    /// `freq_mppb` is the total frequency offset of the local clock in milli-ppb, positive when it
    /// gains — the period is stretched by it so the output keeps true seconds. `lateness_ns` is how
    /// far past the UTC second the next edge would fall.
    pub fn step(&mut self, freq_mppb: i64, lateness_ns: i64) -> PpsStep {
        let correction = (lateness_ns / self.cfg.phase_gain_inv)
            .clamp(-self.cfg.max_correction_ns, self.cfg.max_correction_ns);
        // Frequency through the dither, phase through the accumulator above: both are fractions of
        // a cycle per second, and neither survives being rounded away every edge.
        let base = self
            .dither
            .next_period(self.clk_hz, freq_mppb, 0, self.high_cycles);
        self.phase_acc += correction * self.clk_hz as i64;
        let corr_cycles = self.phase_acc.div_euclid(1_000_000_000);
        self.phase_acc = self.phase_acc.rem_euclid(1_000_000_000);
        let period_word = (base as i64 - corr_cycles) as u32;
        self.edge_ns = self.edge_after(self.edge_ns, period_word);
        PpsStep {
            period_word,
            edge_ns: self.edge_ns,
            correction_ns: correction,
        }
    }

    fn cycles_to_ns(&self, cycles: u64) -> i64 {
        (cycles as u128 * 1_000_000_000 / self.clk_hz as u128) as i64
    }
}

/// Local time of the very first edge, given when the state machine was enabled.
///
/// The program runs `pull`, `mov isr`, `set pindirs`, `pull`, `mov y`, `mov x` and then raises the
/// pin, so the edge is [`OUTPUT_OVERHEAD_CYCLES`] cycles after the enable — 56 ns at 125 MHz, which
/// is far below what the enable can be timestamped to anyway.
pub fn first_edge_ns(clk_hz: u32, enable_ns: i64) -> i64 {
    enable_ns + (OUTPUT_OVERHEAD_CYCLES as u64 * 1_000_000_000 / clk_hz as u64) as i64
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::output_period_cycles;

    const CLK: u32 = 125_000_000;
    /// 100 ms, the width a GPS module's 1PPS has.
    const HIGH: u32 = CLK / 10;

    fn schedule(edge_ns: i64) -> PpsSchedule {
        PpsSchedule::new(CLK, HIGH, PpsScheduleConfig::default(), edge_ns)
    }

    #[test]
    fn a_nominal_word_advances_the_edge_by_one_second() {
        let s = schedule(0);
        assert_eq!(s.edge_after(0, output_period_cycles(CLK, HIGH)), 1_000_000_000);
    }

    #[test]
    fn an_edge_already_on_the_second_stays_where_it_is() {
        let mut s = schedule(0);
        let step = s.step(0, 0);
        assert_eq!(step.correction_ns, 0);
        assert_eq!(step.edge_ns, 1_000_000_000);
    }

    #[test]
    fn a_late_edge_is_pulled_back_and_converges() {
        // The edge is a full millisecond past the second, and the clock is true.
        let mut s = schedule(0);
        let mut lateness = 1_000_000i64;
        for _ in 0..64 {
            let before = s.edge_ns();
            let step = s.step(0, lateness);
            // Whatever the loop took out of the period is what the edge moved by, against the
            // nominal second. It is not quite the correction that was asked for: a period is a
            // whole number of cycles, so the phase lands on an 8 ns grid at 125 MHz.
            let moved = 1_000_000_000 - (step.edge_ns - before);
            assert!(
                (moved - step.correction_ns).abs() <= 1_000_000_000 / CLK as i64,
                "asked {} ns, moved {moved} ns",
                step.correction_ns
            );
            lateness -= moved;
        }
        // Without the carry this stalls at four cycles - the gain reciprocal times the 8 ns
        // a whole cycle is worth.
        assert!(lateness.abs() < 10, "still {lateness} ns late");
    }

    #[test]
    fn one_period_is_only_bent_so_far() {
        // Half a second of error would be half a second of period if it went in whole.
        let mut s = schedule(0);
        let step = s.step(0, 500_000_000);
        assert_eq!(step.correction_ns, PpsScheduleConfig::default().max_correction_ns);
    }

    #[test]
    fn a_gaining_clock_gets_a_longer_period() {
        // 20 ppm fast: a true second needs 20 us more of this clock's cycles.
        let mut fast = schedule(0);
        let mut true_clock = schedule(0);
        let a = fast.step(20_000_000, 0);
        let b = true_clock.step(0, 0);
        assert_eq!(a.period_word - b.period_word, CLK / 50_000);
    }

    #[test]
    fn the_frequency_correction_resolves_finer_than_a_cycle() {
        // One cycle is 8 ns at 125 MHz, i.e. 8 ppb; a 1 ppb offset is a fraction of a cycle per
        // second and has to be carried across edges rather than rounded away.
        let mut s = schedule(0);
        let nominal = output_period_cycles(CLK, HIGH) as i64;
        let mut total = 0i64;
        for _ in 0..1000 {
            total += s.step(1_000, 0).period_word as i64 - nominal;
        }
        // 1000 seconds at 1 ppb is 1 us, which is 125 cycles.
        assert!((total - 125).abs() <= 1, "carried {total} cycles");
    }

    #[test]
    fn the_schedule_starts_from_the_second_edge() {
        // The word pushed at construction governs the first interval, so the edge already fixed
        // when the state machine is enabled is the one a second later.
        let s = PpsSchedule::at_enable(
            CLK,
            HIGH,
            PpsScheduleConfig::default(),
            1_000,
            output_period_cycles(CLK, HIGH),
        );
        assert_eq!(s.edge_ns(), 1_000 + 56 + 1_000_000_000);
    }

    #[test]
    fn acquisition_puts_the_edge_on_the_second_in_one_period() {
        for lateness in [1_000, -1_000, 400_000_000, -400_000_000, 0] {
            let mut s = schedule(0);
            let step = s.acquire(0, lateness);
            // Where the edge would have been, less where it went: what is left has to be a whole
            // second, i.e. still on the boundary.
            let landed = step.edge_ns - 1_000_000_000 + lateness;
            assert!(
                landed.rem_euclid(1_000_000_000).min(
                    1_000_000_000 - landed.rem_euclid(1_000_000_000)
                ) <= 1_000_000_000 / CLK as i64,
                "lateness {lateness} landed at {landed}"
            );
        }
    }

    #[test]
    fn acquisition_never_asks_for_a_period_it_cannot_count() {
        // A period word is a down-counter, so it cannot go negative, and a whole second of delay
        // is the most acquisition ever adds.
        let mut s = schedule(0);
        let step = s.acquire(0, 999_999_999);
        assert!(step.period_word < 2 * CLK, "word {}", step.period_word);
    }

    #[test]
    fn the_first_edge_is_one_pass_of_the_program_after_the_enable() {
        assert_eq!(first_edge_ns(CLK, 1_000), 1_056);
    }
}
