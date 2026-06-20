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
}
