#![cfg_attr(not(test), no_std)]
//! `rp-pps`: RP2040/RP2350 PIO building blocks for a GNSS 1PPS timebase.
//!
//! This is the hardware companion to [`gnssdo`](https://docs.rs/gnssdo): `gnssdo` is the
//! HAL-agnostic discipline core that turns integer-nanosecond timestamps into disciplined UTC,
//! and `rp-pps` is what produces those timestamps (and a steerable 1PPS output) on the RP2040's
//! PIO. The PIO hardware latches the PPS edge with ~16 ns resolution, free of the µs-scale jitter
//! a software GPIO interrupt has on a Cortex-M0+.
//!
//! # Layers
//!
//! - **HAL-agnostic core** (this module, always available): the PIO programs
//!   ([`pps_capture_program`], [`pps_output_program`]), their FIFO-word contracts, and the pure
//!   tick↔ns / period-word math ([`interval_ns`], [`output_period_cycles_ppb`], …). No HAL
//!   dependency, so it is `cargo test`-ed on the host. The programs are built with `pio::pio_asm!`
//!   (not a HAL's re-export), so every backend loads the same [`pio::Program`].
//! - **Backends** (thin, feature-gated): `embassy-rp` (async) and `rp2040-hal` (blocking/IRQ).
//!   Each only loads a core program and moves one FIFO word per second — there is no unified
//!   HAL trait, just a small concrete type per backend.
//!
//! # Scope
//!
//! `rp-pps` owns the *I/O*: capturing edges and emitting pulses. It deliberately does **not** own
//! the discipline (frequency estimation, holdover — that is `gnssdo`) nor the phase servo
//! (PI/PID/Smith control of the output) — those stay in the application. [`output_period_cycles_ppb`]
//! is the generator *protocol* (what word to push for a given frequency offset), not a servo.

use pio::Program;

/// `embassy-rp` (async) backend — [`embassy::PpsCapture`] / [`embassy::PpsOutput`].
#[cfg(feature = "embassy-rp")]
pub mod embassy;

/// `rp2040-hal` (blocking) backend — [`rp2040::PpsCapture`] / [`rp2040::PpsOutput`].
#[cfg(feature = "rp2040-hal")]
pub mod rp2040;

/// One capture tick = 2 PIO clock cycles: [`pps_capture_program`] advances its free-running
/// counter once per 2 cycles (`jmp x--` in a 2-cycle loop), so at 125 MHz one tick is 16 ns.
/// This is a property of the program; the tests assert the program shape that guarantees it.
pub const CAPTURE_CYCLES_PER_TICK: u32 = 2;

/// Fixed per-iteration overhead (PIO clock cycles) of [`pps_output_program`]: the instructions
/// other than the `jmp y-- delay` countdown. The period word pushed to the SM is
/// `clk_cycles_for_one_second - OUTPUT_OVERHEAD_CYCLES` (see [`output_period_cycles`]). It is tied
/// to this exact program; the tests guard the program shape so a change can't silently desync it.
pub const OUTPUT_OVERHEAD_CYCLES: u32 = 10;

/// PPS **input-capture** program for one state machine.
///
/// A free-running down-counter lives in scratch `X`. On the configured `jmp pin`'s rising edge the
/// counter value is pushed to the RX FIFO; the ~2³²-cycle counter wrap (≈ 68 s at 125 MHz) is
/// rejected *inside* the PIO so it never appears as a false edge.
///
/// **Backend setup contract**: configure the input pin as the SM's `jmp` pin, set it as an input,
/// and leave autopush off (the program pushes explicitly).
///
/// **FIFO contract**: each rising edge pushes the 32-bit down-counter value. The interval between
/// two edges is `prev.wrapping_sub(curr)` ticks — see [`interval_ticks`] / [`interval_ns`].
pub fn pps_capture_program() -> Program<32> {
    pio::pio_asm!(
        ".wrap_target",
        "low:",
        "    jmp pin rising", // pin high → real rising edge → capture
        "    jmp x-- low",    // X--; loop while X != 0
        "    jmp low",        // X wrapped to 0: keep going, don't emit a false capture
        "rising:",
        "    in x, 32",
        "    push noblock",
        "high:",
        "    jmp x-- highchk",
        "highchk:",
        "    jmp pin high", // wait for the pin to fall before arming the next edge
        ".wrap",
    )
    .program
}

/// Steerable **1PPS output** program for one state machine.
///
/// Each iteration pulls a fresh *period word* from the TX FIFO (`pull noblock`, so if the FIFO is
/// empty the previously latched period is reused — the output free-runs at the last commanded
/// rate), emits a rising edge plus a short high pulse, then counts the period down.
///
/// **Backend setup contract**: configure the output pin as the SM's `set` pin. Push an initial
/// period word before enabling (e.g. [`output_period_cycles`]).
///
/// **FIFO contract**: push a period word = the low-phase length in PIO clock cycles. Compute it
/// with [`output_period_cycles`] (nominal 1 Hz) or [`output_period_cycles_ppb`] (frequency-corrected).
pub fn pps_output_program() -> Program<32> {
    pio::pio_asm!(
        "    set pindirs, 1", // drive the SET pin as output (once at start)
        ".wrap_target",
        "    pull noblock", // OSR = new period, or the held period if the FIFO is empty
        "    mov x, osr",   // X = period (kept; the countdown uses a copy)
        "    mov y, osr",   // Y = countdown copy
        "    set pins, 1 [10]", // rising edge + ~88 ns high
        "    set pins, 0",  // falling edge
        "delay:",
        "    jmp y-- delay", // hold low for the rest of the period
        ".wrap",
    )
    .program
}

/// Capture interval in ticks between two raw down-counter values, handling the 32-bit wrap.
/// The program counts *down*, so the interval is `prev - curr` (wrapping).
pub fn interval_ticks(prev: u32, curr: u32) -> u32 {
    prev.wrapping_sub(curr)
}

/// Capture interval in nanoseconds between two raw counter values at a given system clock.
///
/// Multiplies before dividing (in `u128`) so there is no per-tick truncation: exact at 125 MHz
/// (1 tick = 16 ns) and as accurate as integer ns allows at any other clock. (Precomputing a
/// `ns_per_tick` and multiplying would accumulate the division error — see [`ns_per_tick`].)
pub fn interval_ns(prev: u32, curr: u32, clk_hz: u32) -> u64 {
    let ticks = interval_ticks(prev, curr) as u128;
    (ticks * CAPTURE_CYCLES_PER_TICK as u128 * 1_000_000_000 / clk_hz as u128) as u64
}

/// Nanoseconds per capture tick at a given system clock (16 at 125 MHz).
///
/// This truncates when `clk_hz` does not divide `CAPTURE_CYCLES_PER_TICK * 1e9` evenly; for an
/// interval, prefer [`interval_ns`], which divides only once over the whole interval.
pub fn ns_per_tick(clk_hz: u32) -> u64 {
    CAPTURE_CYCLES_PER_TICK as u64 * 1_000_000_000 / clk_hz as u64
}

/// One captured edge, timed against the previous one (see [`PpsEdgeTimeline`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimedEdge {
    /// The raw down-counter value at this edge.
    pub raw: u32,
    /// Nanoseconds since the previous edge (0 on the first edge).
    pub interval_ns: u64,
    /// Running nanoseconds since the first observed edge (a continuous capture timeline).
    pub edge_ns: u64,
}

/// Turns a stream of raw capture-counter values into timed edges: it keeps the previous value,
/// computes the [`interval_ns`] (wrap-handled), and accumulates a running `edge_ns` timeline. This
/// is the small bit of capture-side bookkeeping every caller would otherwise hand-roll; it knows
/// nothing about discipline (feed `edge_ns` / `interval_ns` to `gnssdo` yourself).
#[derive(Debug)]
pub struct PpsEdgeTimeline {
    clk_hz: u32,
    last: Option<u32>,
    edge_ns: u64,
}

impl PpsEdgeTimeline {
    /// Create for a given system clock.
    pub const fn new(clk_hz: u32) -> Self {
        Self {
            clk_hz,
            last: None,
            edge_ns: 0,
        }
    }

    /// Record a raw capture-counter value (e.g. from `wait_edge()`); returns the interval since the
    /// previous edge and the running timeline. The first call returns `interval_ns = edge_ns = 0`.
    pub fn observe(&mut self, raw: u32) -> TimedEdge {
        let interval_ns = match self.last {
            Some(prev) => interval_ns(prev, raw, self.clk_hz),
            None => 0,
        };
        self.last = Some(raw);
        self.edge_ns += interval_ns;
        TimedEdge {
            raw,
            interval_ns,
            edge_ns: self.edge_ns,
        }
    }
}

/// Capture ticks in one second at a given system clock (`clk_hz / CAPTURE_CYCLES_PER_TICK`;
/// 62_500_000 at 125 MHz). A property of [`pps_capture_program`]; used to fold a phase to ±½ s.
pub fn capture_ticks_per_second(clk_hz: u32) -> u32 {
    clk_hz / CAPTURE_CYCLES_PER_TICK
}

/// Phase, in capture ticks, of an output edge relative to a reference edge — both captured on the
/// free-running down-counter of [`pps_capture_program`] (on two state machines), given the
/// calibrated constant offset between those counters ([`calibrate_loopback_offset`]). Folded to
/// ±½ second.
///
/// For a 1PPS loopback the two edges fall within the same second, so `reference − output − offset`
/// fits a signed 32-bit tick count (< 2³¹ ticks ≈ 34 s at 125 MHz); larger separations aren't
/// meaningful here.
pub fn loopback_phase_ticks(
    reference_capture: u32,
    output_capture: u32,
    offset_ticks: u32,
    ticks_per_second: u32,
) -> i32 {
    let elapsed = reference_capture
        .wrapping_sub(output_capture)
        .wrapping_sub(offset_ticks) as i32;
    let m = ticks_per_second as i32;
    let r = elapsed % m;
    if r > m / 2 {
        r - m
    } else if r < -m / 2 {
        r + m
    } else {
        r
    }
}

/// Phase, in nanoseconds, of an output 1PPS edge relative to the reference edge:
/// [`loopback_phase_ticks`] converted to ns (multiply-before-divide, exact at 125 MHz where one
/// tick is 16 ns). Feed this to a phase servo (e.g. `gnssdo`'s `PhaseLockLoop`).
pub fn loopback_phase_ns(
    reference_capture: u32,
    output_capture: u32,
    offset_ticks: u32,
    clk_hz: u32,
) -> i64 {
    let ticks = loopback_phase_ticks(
        reference_capture,
        output_capture,
        offset_ticks,
        capture_ticks_per_second(clk_hz),
    ) as i128;
    (ticks * CAPTURE_CYCLES_PER_TICK as i128 * 1_000_000_000 / clk_hz as i128) as i64
}

/// Calibrate the constant counter offset between two capture state machines from samples of the
/// **same** edge captured on both — `(reference_capture, output_capture)` pairs — as the mean of
/// `reference − output` in ticks. `None` if there are no samples. Run once at start-up with both
/// SMs pointed at the reference, then pass the result as `offset_ticks` to [`loopback_phase_ns`].
pub fn calibrate_loopback_offset<I: IntoIterator<Item = (u32, u32)>>(samples: I) -> Option<u32> {
    let (mut sum, mut n): (i64, i64) = (0, 0);
    for (reference, output) in samples {
        sum += reference.wrapping_sub(output) as i32 as i64;
        n += 1;
    }
    if n == 0 {
        None
    } else {
        Some((sum / n) as i32 as u32)
    }
}

/// Nominal output period word for exactly 1 Hz at a given system clock (no frequency correction).
pub fn output_period_cycles(clk_hz: u32) -> u32 {
    clk_hz - OUTPUT_OVERHEAD_CYCLES
}

/// Output period word corrected for a crystal frequency offset (`ppb`, as estimated by
/// [`gnssdo`](https://docs.rs/gnssdo)). To emit a true one-second period, the count is stretched by
/// `clk_hz * ppb / 1e9` cycles. Resolution is one cycle (≈ 8 ppb at 125 MHz); finer steering needs
/// the caller's own sub-cycle dithering.
pub fn output_period_cycles_ppb(clk_hz: u32, ppb: i64) -> u32 {
    let clk = clk_hz as i64;
    let adj = clk * ppb / 1_000_000_000;
    (clk - OUTPUT_OVERHEAD_CYCLES as i64 + adj) as u32
}

/// Sigma-delta period-word generator with sub-cycle frequency resolution.
///
/// [`output_period_cycles_ppb`] quantizes to whole PIO clock cycles (≈8 ppb at 125 MHz), which on
/// a tight loop shows up as a frequency limit cycle. This carries the fractional cycle across edges
/// (first-order sigma-delta) so the *average* output frequency resolves finer than one cycle. It is
/// the generator-protocol counterpart to the control loop: feed it the total frequency offset in
/// milli-ppb (the crystal estimate plus any servo trim) and the servo's immediate phase correction;
/// it returns the next period word for the output state machine.
///
/// Frequency is in **milli-ppb** to match [`gnssdo`](https://docs.rs/gnssdo)'s
/// `PhaseLockLoop` trim resolution; `freq_mppb = crystal_ppb * 1000 + freq_trim_mppb`.
#[derive(Debug, Default)]
pub struct OutputPeriodDither {
    frac_acc: i64, // carried fractional cycles, scaled by 1e12
}

impl OutputPeriodDither {
    /// Create a generator (fraction accumulator starts at 0).
    pub const fn new() -> Self {
        Self { frac_acc: 0 }
    }

    /// Next period word for one output edge. `freq_mppb` is the total frequency offset in milli-ppb
    /// (lengthen the period to compensate a fast crystal); `phase_corr_ns` is the immediate phase
    /// nudge to subtract this edge (e.g. [`gnssdo`](https://docs.rs/gnssdo)
    /// `PhaseLockLoopUpdate::phase_corr_ns`).
    pub fn next_period(&mut self, clk_hz: u32, freq_mppb: i64, phase_corr_ns: i64) -> u32 {
        let clk = clk_hz as i64;
        // Accumulate clk * freq at 1e12 scale (milli-ppb = ppb*1000, ppb = 1e-9), carry the fraction.
        self.frac_acc += clk * freq_mppb;
        let freq_cycles = self.frac_acc.div_euclid(1_000_000_000_000);
        self.frac_acc = self.frac_acc.rem_euclid(1_000_000_000_000);
        let period =
            clk - OUTPUT_OVERHEAD_CYCLES as i64 + freq_cycles - phase_corr_ns * clk / 1_000_000_000;
        period as u32
    }
}

/// Non-blocking capture read, shared across backends (the PIO RX-FIFO contract) so a control loop
/// can be written generic over the HAL. The `embassy` backend additionally offers an `async`
/// `wait_edge()`, intentionally not part of this trait — async and blocking don't share one method
/// cleanly, and forcing them into one would make the trait larger than it honestly is.
pub trait PpsCaptureRead {
    /// The raw down-counter value at the latest captured edge, or `None` if none since the last
    /// read. Feed consecutive values to [`interval_ns`].
    fn try_read(&mut self) -> Option<u32>;
}

/// Commit 1PPS output period words, shared across backends (the PIO TX-FIFO contract).
pub trait PpsPeriodSet {
    /// Push the next period word; returns `false` if the TX FIFO was full. Compute the word with
    /// [`output_period_cycles`] / [`output_period_cycles_ppb`].
    fn set_period(&mut self, period_word: u32) -> bool;
}

/// Steer a 1PPS output by frequency + phase, shared across backends. The easy-tier counterpart to
/// [`PpsPeriodSet`]: the implementor owns the [`OutputPeriodDither`] and system clock, so a control
/// loop can drive the output with only servo quantities and stay generic over the HAL. Implemented
/// by the `SteeredPpsOutput` of each backend.
///
/// There is deliberately no capture-side analogue: the timed-edge read splits into an `async`
/// `next_edge()` (embassy) and a non-blocking `try_timed_edge()` (rp2040-hal), which — like
/// [`PpsCaptureRead`]'s note on `wait_edge` — don't share one method cleanly. The HAL-generic
/// primitive there is [`PpsCaptureRead`] plus [`PpsEdgeTimeline::observe`] on top.
pub trait PpsSteer {
    /// Compute the next period word from the total frequency offset `freq_mppb` (milli-ppb,
    /// `crystal_ppb * 1000 + servo_trim_mppb`) and the immediate `phase_corr_ns` nudge, commit it to
    /// the output, and return it. The push silently drops if the TX FIFO is full (the program holds
    /// the previous period).
    fn set_next_period(&mut self, freq_mppb: i64, phase_corr_ns: i64) -> u32;
}

#[cfg(test)]
mod tests {
    use super::*;

    // Guard the program shapes: CAPTURE_CYCLES_PER_TICK / OUTPUT_OVERHEAD_CYCLES are tied to these
    // exact instruction sequences, so a change must update both the program and the constant.
    #[test]
    fn capture_program_shape() {
        assert_eq!(pps_capture_program().code.len(), 7);
    }

    #[test]
    fn output_program_shape() {
        assert_eq!(pps_output_program().code.len(), 7);
    }

    #[test]
    fn interval_ticks_handles_wrap() {
        assert_eq!(interval_ticks(100, 90), 10);
        // down-counter wrapped past zero between edges
        assert_eq!(interval_ticks(5, u32::MAX - 4), 10);
    }

    #[test]
    fn interval_ns_is_exact_at_125mhz() {
        // 62_500_000 ticks = one second at 125 MHz (62.5M ticks * 16 ns)
        assert_eq!(interval_ns(62_500_000, 0, 125_000_000), 1_000_000_000);
        assert_eq!(interval_ns(1, 0, 125_000_000), 16);
    }

    #[test]
    fn ns_per_tick_at_125mhz() {
        assert_eq!(ns_per_tick(125_000_000), 16);
    }

    #[test]
    fn output_period_nominal() {
        assert_eq!(output_period_cycles(125_000_000), 124_999_990);
    }

    #[test]
    fn output_period_ppb_steers_by_clk_times_ppb() {
        assert_eq!(output_period_cycles_ppb(125_000_000, 0), 124_999_990);
        // +8 ppb at 125 MHz = +1 cycle, -8 ppb = -1 cycle
        assert_eq!(output_period_cycles_ppb(125_000_000, 8), 124_999_991);
        assert_eq!(output_period_cycles_ppb(125_000_000, -8), 124_999_989);
    }

    #[test]
    fn dither_matches_whole_cycle_steering() {
        let mut d = OutputPeriodDither::new();
        // 0 ppb -> nominal; +8 ppb (= 8000 mppb) -> +1 cycle, like output_period_cycles_ppb.
        assert_eq!(d.next_period(125_000_000, 0, 0), 124_999_990);
        assert_eq!(d.next_period(125_000_000, 8_000, 0), 124_999_991);
    }

    #[test]
    fn dither_resolves_sub_cycle_on_average() {
        let mut d = OutputPeriodDither::new();
        // 4 ppb at 125 MHz = half a cycle/s: the fraction carries, so it alternates +0,+1,+0,+1...
        let p0 = d.next_period(125_000_000, 4_000, 0);
        let p1 = d.next_period(125_000_000, 4_000, 0);
        assert_eq!(p0, 124_999_990);
        assert_eq!(p1, 124_999_991);
        // average = 124_999_990.5 = nominal + 0.5 cycle (sub-cycle resolution)
    }

    #[test]
    fn ticks_per_second_at_125mhz() {
        assert_eq!(capture_ticks_per_second(125_000_000), 62_500_000);
    }

    #[test]
    fn loopback_phase_zero_when_aligned() {
        // output edge exactly `offset` ticks behind the reference → phase 0.
        let k = 1000;
        let gps = 5_000_000u32;
        let out = gps.wrapping_sub(k);
        assert_eq!(loopback_phase_ticks(gps, out, k, 62_500_000), 0);
        assert_eq!(loopback_phase_ns(gps, out, k, 125_000_000), 0);
    }

    #[test]
    fn loopback_phase_lead_and_lag() {
        // reference 100 ticks ahead of output → +100 ticks = +1600 ns at 125 MHz.
        assert_eq!(loopback_phase_ticks(100, 0, 0, 62_500_000), 100);
        assert_eq!(loopback_phase_ns(100, 0, 0, 125_000_000), 1600);
        // output ahead of reference → negative.
        assert_eq!(loopback_phase_ticks(0, 100, 0, 62_500_000), -100);
        assert_eq!(loopback_phase_ns(0, 100, 0, 125_000_000), -1600);
    }

    #[test]
    fn loopback_phase_folds_to_half_second() {
        let tps = 62_500_000;
        // just under a full second wraps to a small negative (the next second's edge).
        assert_eq!(loopback_phase_ticks(tps - 10, 0, 0, tps), -10);
        // just over half a second folds to the negative side.
        assert_eq!(
            loopback_phase_ticks(tps / 2 + 5, 0, 0, tps),
            (tps / 2 + 5) as i32 - tps as i32
        );
    }

    #[test]
    fn calibrate_offset_averages_same_edge_diffs() {
        // constant reference−output diff of 1000 ticks → offset 1000.
        assert_eq!(
            calibrate_loopback_offset([(5000u32, 4000u32), (8000, 7000), (12345, 11345)]),
            Some(1000)
        );
        let empty: [(u32, u32); 0] = [];
        assert_eq!(calibrate_loopback_offset(empty), None);
    }

    #[test]
    fn dither_phase_corr_subtracts_cycles() {
        let mut d = OutputPeriodDither::new();
        // phase_corr 8 ns at 125 MHz = 1 cycle subtracted; freq 0.
        assert_eq!(d.next_period(125_000_000, 0, 8), 124_999_989);
        // negative phase_corr lengthens.
        let mut d2 = OutputPeriodDither::new();
        assert_eq!(d2.next_period(125_000_000, 0, -8), 124_999_991);
    }

    #[test]
    fn edge_timeline_first_edge_is_zero() {
        let mut t = PpsEdgeTimeline::new(125_000_000);
        let e = t.observe(1000);
        assert_eq!(
            e,
            TimedEdge {
                raw: 1000,
                interval_ns: 0,
                edge_ns: 0
            }
        );
    }

    #[test]
    fn edge_timeline_accumulates_1hz() {
        let mut t = PpsEdgeTimeline::new(125_000_000);
        // down-counter: 62.5M ticks/s; first edge then two ~1 s edges (wrap-handled).
        t.observe(0xFFFF_FFFF);
        let e1 = t.observe(0xFFFF_FFFF - 62_500_000);
        assert_eq!(e1.interval_ns, 1_000_000_000);
        assert_eq!(e1.edge_ns, 1_000_000_000);
        let e2 = t.observe(0xFFFF_FFFF - 125_000_000);
        assert_eq!(e2.interval_ns, 1_000_000_000);
        assert_eq!(e2.edge_ns, 2_000_000_000);
    }
}
