#![cfg_attr(not(test), no_std)]
//! `rp-pps`: RP2040/RP2350 PIO building blocks for a GNSS 1PPS timebase, plus NMEA time ingestion.
//!
//! This is the device/receiver-facing companion to [`gnssdo`](https://docs.rs/gnssdo): `gnssdo` is
//! the HAL-agnostic discipline core that turns timestamps + a UTC epoch into disciplined UTC, and
//! `rp-pps` is what *produces* those inputs. It hardware-timestamps the PPS edge on the RP2040's
//! PIO (~16 ns, free of the µs-scale jitter a software GPIO interrupt has on a Cortex-M0+), emits a
//! steerable 1PPS, and decodes the receiver's NMEA to pair each edge with its UTC second.
//!
//! # Layers
//!
//! - **HAL-agnostic core** (always available, host-tested): the PIO programs
//!   ([`pps_capture_program`], [`pps_output_program`]) and their FIFO-word contracts; the tick↔ns
//!   / period-word math ([`interval_ns`], [`output_period_cycles_ppb`], …); NMEA framing/parsing
//!   ([`NmeaLineAssembler`], [`parse_rmc_time_date`]); and the PPS↔UTC-second pairing
//!   ([`PpsTimeSync`]). No HAL dependency. The programs are built with `pio::pio_asm!` (not a HAL's
//!   re-export), so every backend loads the same [`pio::Program`].
//! - **Backends** (thin, feature-gated): `embassy-rp` (async) and `rp2040-hal` (blocking/IRQ).
//!   Each only loads a core program and moves one FIFO word per second — there is no unified HAL
//!   trait, just a small concrete type per backend.
//!
//! # Scope
//!
//! `rp-pps` owns the *device/receiver I/O and time ingestion*: capturing edges, emitting pulses,
//! and turning the receiver's NMEA + a PPS edge into a UTC epoch. It deliberately does **not** own
//! the discipline (frequency estimation, holdover, the phase servo) — that is [`gnssdo`](https://docs.rs/gnssdo)'s
//! job; feed it the timestamps and epoch this crate produces. [`output_period_cycles_ppb`] is the
//! generator *protocol* (what word to push for a given frequency offset), not a servo.
//!
//! # Features
//!
//! - **`external-nmea`** (off by default): parse RMC with the [`nmea`](https://docs.rs/nmea) crate
//!   instead of the zero-dependency built-in parser. See [`parse_rmc_time_date`] for the behavioural
//!   differences (checksum validation, year pivot, leap-second handling).

use pio::Program;

/// `embassy-rp` (async) backend — [`embassy::PpsCapture`] / [`embassy::PpsOutput`].
#[cfg(feature = "embassy-rp")]
pub mod embassy;

/// `rp2040-hal` (blocking) backend — [`rp2040::PpsCapture`] / [`rp2040::PpsOutput`].
#[cfg(feature = "rp2040-hal")]
pub mod rp2040;

mod assembler;
mod schedule;
mod timesync;

pub use schedule::{PpsSchedule, PpsScheduleConfig, PpsStep, first_edge_ns};

pub use assembler::{MAX_SENTENCE_LEN, NmeaLineAssembler, nmea_checksum, nmea_checksum_valid};
pub use timesync::{
    PpsNmeaAssociation, PpsTimeSync, RmcTimeDate, SyncEpoch, civil_to_unix, days_from_civil,
    parse_ddmmyy, parse_hhmmss, parse_rmc_time_date, parse_zda_time_date,
};

/// Turn-key GPSDO state bundle (PPS edge + NMEA → disciplined UTC). Enable with the `gnssdo` feature.
#[cfg(feature = "gnssdo")]
mod gpsdo;
#[cfg(feature = "gnssdo")]
pub use gpsdo::{NmeaTimeSource, PpsGpsdo, SyncReport};

/// One capture tick = 2 PIO clock cycles: [`pps_capture_program`] advances its free-running
/// counter once per 2 cycles (`jmp x--` in a 2-cycle loop), so at 125 MHz one tick is 16 ns.
/// This is a property of the program; the tests assert the program shape that guarantees it.
pub const CAPTURE_CYCLES_PER_TICK: u32 = 2;

/// Fixed per-iteration overhead (PIO clock cycles) of [`pps_output_program`]: the instructions
/// other than the two countdown loops (`jmp x-- high` and `jmp y-- low`). The low-phase period word
/// pushed to the SM is `clk_cycles_for_one_second - OUTPUT_OVERHEAD_CYCLES - high_cycles` (see
/// [`output_period_cycles`]). It is tied to this exact program; the tests guard the program shape so
/// a change can't silently desync it.
pub const OUTPUT_OVERHEAD_CYCLES: u32 = 7;

/// Which level a receiver's 1PPS asserts, and therefore which edge marks the second.
///
/// [`pps_capture_program`] watches for a **rising** edge, so a receiver that idles high and pulses
/// low has to have its pin inverted on the way into PIO. Without that the capture lands on the
/// *end* of the pulse — one pulse width past the second, typically 100 ms — and nothing downstream
/// can tell, because every interval is still exactly one second and the output still locks to the
/// input within nanoseconds. Only a comparison against an outside clock shows it.
///
/// Polarity is a property of the receiver and the board it sits on, so it belongs in the firmware's
/// configuration — but finding out what to configure can take work. The AE-GNSS-EXTANT board this
/// project is built around documents `1PPS 出力 : C-MOS ロジック (3.3V) レベル,
/// パルス幅 :100mS (アクティブ Low)`, while the MediaTek software specifications underneath it
/// describe the pulse against its *rising* edge and state no polarity anywhere; the two agree only
/// once the schematic shows the 1PPS passing through one gate of a 74HC04. Where the documents run
/// out, [`PolarityProbe`] reads it off the pin.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum PpsPolarity {
    /// Idle low, pulses high. The rising edge marks the second, which is what the capture program
    /// already looks for.
    #[default]
    ActiveHigh,
    /// Idle high, pulses low. The falling edge marks the second, so the input has to be inverted.
    ActiveLow,
}

/// Decide a [`PpsPolarity`] by counting how much of a second the pin spends high.
///
/// A 1PPS is one short excursion per second, so the level held for most of the second is the idle
/// level and the short one is the pulse. Feed samples taken at a steady rate over more than a full
/// second — both levels have to appear or there is nothing to compare.
///
/// A bring-up tool, not a boot step. It can only answer while a pulse is running — a receiver
/// without a fix drives none — and a resting pin yields an answer that looks measured and is a
/// guess, which is what [`Self::saw_pulse`] is for. Once the answer is known it is a fact about the
/// hardware, so it belongs in a constant rather than in a window the firmware repeats every boot.
#[derive(Clone, Copy, Default, Debug)]
pub struct PolarityProbe {
    high: u32,
    total: u32,
}

impl PolarityProbe {
    pub const fn new() -> Self {
        Self { high: 0, total: 0 }
    }

    /// Record one reading of the pin.
    pub fn sample(&mut self, pin_high: bool) {
        self.total = self.total.saturating_add(1);
        if pin_high {
            self.high = self.high.saturating_add(1);
        }
    }

    /// How many samples have been taken.
    pub fn samples(&self) -> u32 {
        self.total
    }

    /// Fraction of samples that were high, in percent. `None` before any sample.
    pub fn duty_percent(&self) -> Option<u32> {
        (self.total != 0).then(|| self.high * 100 / self.total)
    }

    /// Whether a pulse was actually seen: both levels appeared in the window.
    ///
    /// A pin held at one level the whole time carries no 1PPS, and [`Self::polarity`] would still
    /// name a polarity for it. Check this before believing that answer.
    pub fn saw_pulse(&self) -> bool {
        self.high != 0 && self.high != self.total
    }

    /// The polarity these samples imply, or `None` before any sample.
    ///
    /// Majority high means the pulse is the low excursion. A pin sitting at one level for the whole
    /// window — a receiver with no fix, or no receiver at all — reports that level's polarity, so
    /// the caller has to know the pulse is present before believing this.
    pub fn polarity(&self) -> Option<PpsPolarity> {
        self.duty_percent().map(|pct| {
            if pct > 50 {
                PpsPolarity::ActiveLow
            } else {
                PpsPolarity::ActiveHigh
            }
        })
    }
}

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

/// Wrap-cost-balanced variant of [`pps_capture_program`].
///
/// In the original program the X counter's 2³²-wrap (every ≈68.7 s at 125 MHz) costs one extra
/// cycle in the **low** wait loop (the `jmp low` guard) but nothing in the **high** wait loop
/// (`jmp x--`'s fall-through already lands on the next instruction). A pair of counters watching
/// pins with *different* duty cycles therefore drifts apart by `wraps/min × Δduty × 8 ns` — about
/// 5.6 ns/min for a 100 ms-high output looped back against a 900 ms-high receiver PPS (measured;
/// the periodic-recalibration `dk ≈ −1 tick / 2.5 min` was exactly this).
///
/// This variant adds the same one-instruction guard to the high loop, so a wrap costs +1 cycle in
/// **both** loops: every counter slips a uniform 8 ns per wrap (≈0.12 ppb) regardless of the
/// waveform it watches. The common slip cancels in every counter-pair difference and is absorbed
/// by frequency estimation; pair drift becomes duty-independent. Tick rate (2 cycles) and the
/// capture-path cost are unchanged. Choose per state machine set — all counters that are compared
/// against each other should run the same variant.
pub fn pps_capture_program_wrap_balanced() -> Program<32> {
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
        "    jmp highchk", // X wrapped to 0: same +1-cycle toll as the low loop (wrap-cost symmetry)
        "highchk:",
        "    jmp pin high", // wait for the pin to fall before arming the next edge
        ".wrap",
    )
    .program
}

/// Steerable **1PPS output** program for one state machine, with a configurable high-pulse width.
///
/// Once at start it pulls a *high-width word* and stashes it in `ISR` (the high pulse length in PIO
/// clock cycles — set it once, e.g. via [`output_high_cycles`]). Then each iteration pulls a fresh
/// *low-period word* from the TX FIFO via `pull noblock`, emits the rising edge, holds
/// high for the stashed width (`jmp x-- high`), drops, and holds low for the rest of the second
/// (`jmp y-- low`). The disciplined quantity is the *rising edge*; the high width is timing-neutral
/// (it is accounted for in the low-period word, so widening the pulse does not move the edge).
///
/// **Empty-FIFO behaviour (important)**: a non-blocking `pull` on an *empty* TX FIFO does **not**
/// hold the last period. Per the RP2040 datasheet (§3.4.7.2) it copies scratch `X` into the OSR
/// (equivalent to `MOV OSR, X`); recycling the last word would require an explicit `MOV X, OSR`
/// after the pull, which this program does not do — and cannot, since `X` = high-phase count and
/// `Y` = low-phase count leave no spare register. At the `pull`, `X` is the *spent* high counter
/// (`0xFFFFFFFF`, having wrapped past 0 in `jmp x-- high`), so an empty pull loads a ~2³²-cycle
/// period (~34 s at 125 MHz): one dropped PPS, not a graceful free-run at the last rate. The caller
/// must therefore push a fresh period on **every** edge; in practice the period is pushed ~1 s ahead
/// of the pull (right after the loopback capture), so the FIFO is non-empty when the SM pulls.
///
/// **Backend setup contract**: configure the output pin as the SM's `set` pin. Push the high-width
/// word first, then an initial low-period word, before enabling (see [`output_high_cycles`] /
/// [`output_period_cycles`]).
///
/// **FIFO contract**: push **one** high-width word at init, then a low-period word per edge. Compute
/// the period with [`output_period_cycles`] (nominal 1 Hz) or [`output_period_cycles_ppb`]
/// (frequency-corrected); both subtract the high width so the rising edge stays on the second.
pub fn pps_output_program() -> Program<32> {
    pio::pio_asm!(
        "    pull block",     // OSR = high-width word (init; blocks until pushed)
        "    mov isr, osr",   // ISR = high-width (persistent stash, never clobbered)
        "    set pindirs, 1", // drive the SET pin as output (once at start)
        ".wrap_target",
        "    pull noblock", // OSR = new low period, or the held period if the FIFO is empty
        "    mov y, osr",   // Y = low-phase countdown
        "    mov x, isr",   // X = high-phase countdown (copy of the stashed width)
        "    set pins, 1",  // rising edge
        "high:",
        "    jmp x-- high", // hold high for the stashed width
        "    set pins, 0",  // falling edge
        "low:",
        "    jmp y-- low", // hold low for the rest of the period
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

    /// Create a timeline whose zero is the moment the state machine started counting.
    ///
    /// [`new`](Self::new) puts zero at the *first edge*, which is all a frequency estimate needs.
    /// Anything that has to be compared with another state machine needs a zero the two share, and
    /// state machines enabled by one write share the instant they started.
    ///
    /// The caller must have left `X` at zero before enabling (inject `mov x, null` while stopped),
    /// because that is what makes the first captured value say how long the counting had been
    /// going: the counter runs down from zero, so `0 - raw` is the elapsed ticks.
    pub const fn from_counter_start(clk_hz: u32) -> Self {
        Self {
            clk_hz,
            // Zero is not "no reading yet" here, it is the reading the counter had when it was
            // enabled. The first capture then measures an interval like any other.
            last: Some(0),
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

/// Raw (un-folded) loopback tick difference `reference − output − offset`, as a signed count. For an
/// **adjacent-edge** pairing (the reference and output captures are from the same second) this is the
/// small true phase; a **mis-pairing** (captures from non-adjacent edges, e.g. after a PPS
/// dropout/drain/relock) makes it ≈ ±`ticks_per_second`. [`loopback_phase_ticks`] folds mod the
/// nominal second, which *hides* such a slip and leaves a ppm×1s residual the servo then locks to
/// (the historic "stale pairing" failure family). Gate the measurement on `|raw| <= max_lag_ticks`
/// (a few ms ≫ any real phase, ≪ 1 s) so only correctly-paired edges drive the loop, keeping
/// `phase == 0 ⟺ aligned`.
pub fn loopback_raw_lag_ticks(
    reference_capture: u32,
    output_capture: u32,
    offset_ticks: u32,
) -> i32 {
    reference_capture
        .wrapping_sub(output_capture)
        .wrapping_sub(offset_ticks) as i32
}

/// Phase, in nanoseconds, of an output 1PPS edge relative to the reference edge:
/// [`loopback_phase_ticks`] converted to ns (multiply-before-divide, exact at 125 MHz where one
/// tick is 16 ns). Feed this to a phase servo (e.g. `gnssdo`'s `PhaseLockLoop`).
///
/// **Edge-definition note**: this phase is defined at the **PIO input switching threshold** of the
/// two captured pins (the digital edge the state machines actually see), *not* at a scope mid-level
/// crossing. An oscilloscope that triggers/measures at a different threshold (e.g. 1.65 V mid-level
/// vs the RP2040 input V_IH) reads a *definitional* offset on top of the real one, which grows when
/// the two signals have different edge slopes. Measured on hardware (sweeping the scope threshold
/// over the RP2040 0.8..2.0 V input band), this definitional term is only ~1.4 ns here: both edges
/// are fast, so the threshold barely moves the crossing. The bulk of the relative GP3-vs-GPS offset
/// is therefore elsewhere (path/pad delays, K residual, probe skew), not the edge definition.
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

/// Fold a sub-second phase difference `out_ns - ref_ns` into `(-½ s, +½ s]` nanoseconds.
///
/// The software (non-PIO) 1PPS path measures the output edge's sub-second position against the
/// reference (GPS) edge's, both on the same local timebase. Either timestamp can be mis-attributed
/// to an adjacent UTC second (a ±1 s output/input race), but the folded sub-second phase is
/// invariant to adding or removing whole seconds from *either* argument — so this internal phase
/// metric needs no same-second pairing (the `C0_GEN`-style coordination the PIO path uses). The
/// upper bound is closed and the lower bound open, matching [`loopback_phase_ticks`]'s half-second
/// fold convention (so `+½ s` reads as a lead, never as `−½ s`).
pub fn fold_phase_ns(out_ns: i64, ref_ns: i64) -> i64 {
    const SEC: i64 = 1_000_000_000;
    let mut d = (out_ns - ref_ns) % SEC;
    if d > SEC / 2 {
        d -= SEC;
    } else if d <= -SEC / 2 {
        d += SEC;
    }
    d
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

/// Drain a capture FIFO to its **most recent** value, returning `(latest, dropped)`.
///
/// The runtime phase loop reads one output-edge capture per second, but a `wait_pull` returns the
/// *oldest* FIFO entry. If the consumer ever falls behind (e.g. a stall during holdover recovery),
/// stale captures pile up and `wait_pull` keeps handing back an N-second-old edge — which, paired
/// with the *current* reference and folded by [`loopback_phase_ticks`] (nominal-second fold),
/// silently injects `N × ppm × 1 s` of phase error while reporting a near-zero residual. Pulling the
/// blocking edge and then draining any backlog here keeps the loop on the current edge.
///
/// `first` is the value already obtained from a blocking `wait_pull`; `try_more` yields each
/// remaining FIFO entry (a non-blocking `try_pull`) until empty. `dropped` (number discarded) is a
/// useful health signal: a persistently non-zero count means the consumer is lagging.
pub fn latest_capture(first: u32, mut try_more: impl FnMut() -> Option<u32>) -> (u32, u32) {
    let (mut latest, mut dropped) = (first, 0u32);
    while let Some(v) = try_more() {
        latest = v;
        dropped += 1;
    }
    (latest, dropped)
}

/// High-pulse width (`high_cycles` for [`pps_output_program`]) for a desired pulse width in
/// nanoseconds at a given system clock. The 1PPS rising edge is what's disciplined; this only sets
/// how long the pin stays high (e.g. ~100 ms is the common GPS-module / GPSDO convention, a few µs
/// suits counters/scopes). Push the returned value once at init; the period helpers below subtract
/// it so the edge timing is unchanged regardless of width.
pub fn output_high_cycles(clk_hz: u32, width_ns: u32) -> u32 {
    (width_ns as u64 * clk_hz as u64 / 1_000_000_000) as u32
}

/// Nominal low-period word for exactly 1 Hz at a given system clock (no frequency correction), for a
/// high pulse of `high_cycles` ([`output_high_cycles`]).
pub fn output_period_cycles(clk_hz: u32, high_cycles: u32) -> u32 {
    clk_hz - OUTPUT_OVERHEAD_CYCLES - high_cycles
}

/// Low-period word corrected for a crystal frequency offset (`ppb`, as estimated by
/// [`gnssdo`](https://docs.rs/gnssdo)), for a high pulse of `high_cycles` ([`output_high_cycles`]).
/// To emit a true one-second period, the count is stretched by `clk_hz * ppb / 1e9` cycles.
/// Resolution is one cycle (≈ 8 ppb at 125 MHz); finer steering needs the caller's own sub-cycle
/// dithering.
pub fn output_period_cycles_ppb(clk_hz: u32, ppb: i64, high_cycles: u32) -> u32 {
    let clk = clk_hz as i64;
    let adj = clk * ppb / 1_000_000_000;
    (clk - OUTPUT_OVERHEAD_CYCLES as i64 - high_cycles as i64 + adj) as u32
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

    /// Next low-period word for one output edge, for a high pulse of `high_cycles`
    /// ([`output_high_cycles`]). `freq_mppb` is the total frequency offset in milli-ppb (lengthen the
    /// period to compensate a fast crystal); `phase_corr_ns` is the immediate phase nudge to subtract
    /// this edge (e.g. [`gnssdo`](https://docs.rs/gnssdo) `PhaseLockLoopUpdate::phase_corr_ns`).
    pub fn next_period(
        &mut self,
        clk_hz: u32,
        freq_mppb: i64,
        phase_corr_ns: i64,
        high_cycles: u32,
    ) -> u32 {
        let clk = clk_hz as i64;
        // Accumulate clk * freq at 1e12 scale (milli-ppb = ppb*1000, ppb = 1e-9), carry the fraction.
        self.frac_acc += clk * freq_mppb;
        let freq_cycles = self.frac_acc.div_euclid(1_000_000_000_000);
        self.frac_acc = self.frac_acc.rem_euclid(1_000_000_000_000);
        let period = clk - OUTPUT_OVERHEAD_CYCLES as i64 - high_cycles as i64 + freq_cycles
            - phase_corr_ns * clk / 1_000_000_000;
        period as u32
    }
}

/// Sigma-delta period generator for the **software (non-PIO) 1PPS path**, in embassy-time ticks.
///
/// The naive output task toggles a GPIO with `Timer::after`, whose quantum is one embassy tick (1 µs
/// on the RP2040). Rounding the per-edge frequency steering to whole ticks would quantize the output
/// frequency to ~1 tick / 1 s = 1 ppm (1000 ppb at a 1 MHz tick) — far coarser than the crystal
/// estimate, so the *average* drift would still be steered only in 1000 ppb steps. This carries the
/// fractional tick across edges (first-order sigma-delta, the embassy-tick twin of
/// [`OutputPeriodDither`]) so the average period resolves sub-tick.
///
/// It steers **frequency only** and returns the full rising-to-rising period; the high-pulse width is
/// the caller's (subtract it from the returned period for the low-phase wait), matching how the PIO
/// program accounts the pulse separately from the disciplined rising edge.
#[derive(Debug, Default)]
pub struct NaivePeriodDither {
    frac_acc: i64, // carried fractional ticks, scaled by 1e12
}

impl NaivePeriodDither {
    /// Create a generator (fraction accumulator starts at 0).
    pub const fn new() -> Self {
        Self { frac_acc: 0 }
    }

    /// Next full rising-to-rising period, in embassy-time ticks, for a total frequency offset
    /// `freq_mppb` (milli-ppb; *lengthen* the period to compensate a fast crystal — the same sign as
    /// [`output_period_cycles_ppb`]). The nominal period is `tick_hz` ticks (one second); the
    /// fractional steering tick is carried so the average frequency resolves below one tick.
    pub fn next_ticks(&mut self, tick_hz: u32, freq_mppb: i64) -> u32 {
        let t = tick_hz as i64;
        // Accumulate tick_hz * freq at 1e12 scale (milli-ppb = ppb*1000, ppb = 1e-9), carry the frac.
        self.frac_acc += t * freq_mppb;
        let freq_ticks = self.frac_acc.div_euclid(1_000_000_000_000);
        self.frac_acc = self.frac_acc.rem_euclid(1_000_000_000_000);
        (t + freq_ticks) as u32
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

    /// One second of samples at `step_ms`, high for `high_ms` of it.
    fn probe_pulse(high_ms: u32, step_ms: u32) -> PolarityProbe {
        let mut probe = PolarityProbe::new();
        for t in (0..1000).step_by(step_ms as usize) {
            probe.sample(t < high_ms);
        }
        probe
    }

    #[test]
    fn a_short_high_pulse_is_active_high() {
        // The shape the capture program already expects: idle low, 100 ms high.
        let probe = probe_pulse(100, 1);
        assert_eq!(probe.polarity(), Some(PpsPolarity::ActiveHigh));
        assert_eq!(probe.duty_percent(), Some(10));
    }

    #[test]
    fn a_short_low_pulse_is_active_low() {
        // The AE-GNSS-EXTANT board: idle high, 100 ms low. Reading this as active-high puts the
        // capture on the end of the pulse, 100 ms past the second.
        let probe = probe_pulse(900, 1);
        assert_eq!(probe.polarity(), Some(PpsPolarity::ActiveLow));
        assert_eq!(probe.duty_percent(), Some(90));
    }

    #[test]
    fn a_wide_pulse_is_still_decided_by_which_excursion_is_shorter() {
        // PMTK285 can widen the pulse. 400 ms high is still the short excursion, so still the mark.
        assert_eq!(
            probe_pulse(400, 1).polarity(),
            Some(PpsPolarity::ActiveHigh)
        );
        // ...and 600 ms high means the 400 ms low is the pulse.
        assert_eq!(probe_pulse(600, 1).polarity(), Some(PpsPolarity::ActiveLow));
    }

    #[test]
    fn a_coarse_sampler_still_gets_the_answer() {
        // The probe runs on whatever the caller can afford. At 20 ms steps a 100 ms pulse is only
        // five samples, and the decision still has to come out right.
        assert_eq!(
            probe_pulse(100, 20).polarity(),
            Some(PpsPolarity::ActiveHigh)
        );
        assert_eq!(
            probe_pulse(900, 20).polarity(),
            Some(PpsPolarity::ActiveLow)
        );
    }

    #[test]
    fn a_pin_that_never_moves_reports_its_level() {
        // No fix, no receiver, or a disconnected pin. There is no pulse to find, so the answer
        // describes the level rather than a 1PPS — the caller has to know a pulse is present.
        let mut stuck_high = PolarityProbe::new();
        let mut stuck_low = PolarityProbe::new();
        for _ in 0..100 {
            stuck_high.sample(true);
            stuck_low.sample(false);
        }
        assert_eq!(stuck_high.polarity(), Some(PpsPolarity::ActiveLow));
        assert_eq!(stuck_low.polarity(), Some(PpsPolarity::ActiveHigh));
    }

    #[test]
    fn nothing_is_claimed_before_the_first_sample() {
        let probe = PolarityProbe::new();
        assert_eq!(probe.polarity(), None);
        assert_eq!(probe.duty_percent(), None);
        assert_eq!(probe.samples(), 0);
    }

    #[test]
    fn a_pulse_is_seen_only_when_both_levels_appear() {
        let mut high_pulse = PolarityProbe::new();
        let mut low_pulse = PolarityProbe::new();
        for i in 0..100 {
            high_pulse.sample(i < 10);
            low_pulse.sample(i >= 10);
        }
        assert!(high_pulse.saw_pulse());
        assert!(low_pulse.saw_pulse());
    }

    #[test]
    fn a_pin_that_never_moves_saw_no_pulse() {
        // The level is all there is to report, so polarity() answers while saw_pulse() denies it.
        let mut stuck_high = PolarityProbe::new();
        let mut stuck_low = PolarityProbe::new();
        for _ in 0..100 {
            stuck_high.sample(true);
            stuck_low.sample(false);
        }
        assert!(!stuck_high.saw_pulse());
        assert!(!stuck_low.saw_pulse());
    }

    #[test]
    fn no_samples_is_no_pulse() {
        assert!(!PolarityProbe::new().saw_pulse());
    }

    #[test]
    fn a_single_sample_is_not_yet_a_pulse() {
        let mut probe = PolarityProbe::new();
        probe.sample(true);
        assert!(!probe.saw_pulse());
    }

    #[test]
    fn a_long_run_does_not_overflow_the_counters() {
        // Sampling for hours must not wrap into a wrong answer.
        let mut probe = PolarityProbe::new();
        probe.high = u32::MAX - 1;
        probe.total = u32::MAX - 1;
        for _ in 0..8 {
            probe.sample(true);
        }
        assert_eq!(probe.samples(), u32::MAX);
    }

    // Guard the program shapes: CAPTURE_CYCLES_PER_TICK / OUTPUT_OVERHEAD_CYCLES are tied to these
    // exact instruction sequences, so a change must update both the program and the constant.
    #[test]
    fn capture_program_shape() {
        assert_eq!(pps_capture_program().code.len(), 7);
    }

    // 対称版: high 待ちループにも 0 跨ぎの受け皿 jmp が入るぶん 1 命令長い。
    // 一周のコストが両ループで +1 cycle に揃い、ペアの読み差が duty 非依存になる。
    #[test]
    fn wrap_balanced_capture_program_shape() {
        assert_eq!(pps_capture_program_wrap_balanced().code.len(), 8);
    }

    #[test]
    fn output_program_shape() {
        assert_eq!(pps_output_program().code.len(), 10);
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
    fn output_high_cycles_from_width() {
        // 100 ms at 125 MHz = 12.5 M cycles; ~88 ns ~= 11 cycles.
        assert_eq!(output_high_cycles(125_000_000, 100_000_000), 12_500_000);
        assert_eq!(output_high_cycles(125_000_000, 88), 11);
    }

    #[test]
    fn output_period_nominal() {
        // No high pulse: clk - overhead.
        assert_eq!(output_period_cycles(125_000_000, 0), 124_999_993);
        // A 100 ms high pulse is subtracted from the low period so the edge stays on the second.
        let high = output_high_cycles(125_000_000, 100_000_000);
        assert_eq!(
            output_period_cycles(125_000_000, high),
            124_999_993 - 12_500_000
        );
    }

    #[test]
    fn output_period_ppb_steers_by_clk_times_ppb() {
        assert_eq!(output_period_cycles_ppb(125_000_000, 0, 0), 124_999_993);
        // +8 ppb at 125 MHz = +1 cycle, -8 ppb = -1 cycle
        assert_eq!(output_period_cycles_ppb(125_000_000, 8, 0), 124_999_994);
        assert_eq!(output_period_cycles_ppb(125_000_000, -8, 0), 124_999_992);
        // high width shifts the low period but not the per-ppb steering.
        assert_eq!(
            output_period_cycles_ppb(125_000_000, 8, 12_500_000),
            124_999_994 - 12_500_000
        );
    }

    #[test]
    fn dither_matches_whole_cycle_steering() {
        let mut d = OutputPeriodDither::new();
        // 0 ppb -> nominal; +8 ppb (= 8000 mppb) -> +1 cycle, like output_period_cycles_ppb.
        assert_eq!(d.next_period(125_000_000, 0, 0, 0), 124_999_993);
        assert_eq!(d.next_period(125_000_000, 8_000, 0, 0), 124_999_994);
    }

    #[test]
    fn dither_resolves_sub_cycle_on_average() {
        let mut d = OutputPeriodDither::new();
        // 4 ppb at 125 MHz = half a cycle/s: the fraction carries, so it alternates +0,+1,+0,+1...
        let p0 = d.next_period(125_000_000, 4_000, 0, 0);
        let p1 = d.next_period(125_000_000, 4_000, 0, 0);
        assert_eq!(p0, 124_999_993);
        assert_eq!(p1, 124_999_994);
        // average = 124_999_993.5 = nominal + 0.5 cycle (sub-cycle resolution)
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
    fn raw_lag_small_for_adjacent_edge_large_for_mispairing() {
        let tps = 62_500_000u32; // 1 s of capture ticks at 125 MHz (÷2)
        let k = 1000u32;
        let gps = 5_000_000u32;
        // adjacent pairing: output ~94 ticks (≈1.5µs) after the reference → raw lag is the small phase.
        let out = gps.wrapping_sub(k).wrapping_sub(94);
        assert_eq!(loopback_raw_lag_ticks(gps, out, k), 94);
        // mis-pairing by one second (output captured a second later) → raw lag ≈ +1 s, which the
        // fold would otherwise hide. Gating |raw| <= a few-ms threshold rejects it.
        let out_next = out.wrapping_sub(tps);
        assert_eq!(loopback_raw_lag_ticks(gps, out_next, k), 94 + tps as i32);
        assert!(loopback_raw_lag_ticks(gps, out_next, k).unsigned_abs() > tps / 100);
        assert!(loopback_raw_lag_ticks(gps, out, k).unsigned_abs() <= tps / 100);
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
    fn latest_capture_drains_to_newest() {
        // backlog of 3 → use the newest (13), report 3 dropped. This is the runtime fix:
        // without draining, wait_pull would hand back the oldest (10) — an N-second-stale edge.
        let mut it = [11u32, 12, 13].into_iter();
        assert_eq!(latest_capture(10, || it.next()), (13, 3));
        // empty FIFO → the blocking value is current, nothing dropped.
        let mut none = core::iter::empty::<u32>();
        assert_eq!(latest_capture(10, || none.next()), (10, 0));
    }

    #[test]
    fn dither_phase_corr_subtracts_cycles() {
        let mut d = OutputPeriodDither::new();
        // phase_corr 8 ns at 125 MHz = 1 cycle subtracted; freq 0, no high pulse.
        assert_eq!(d.next_period(125_000_000, 0, 8, 0), 124_999_992);
        // negative phase_corr lengthens.
        let mut d2 = OutputPeriodDither::new();
        assert_eq!(d2.next_period(125_000_000, 0, -8, 0), 124_999_994);
    }

    #[test]
    fn naive_dither_matches_whole_tick_steering() {
        let mut d = NaivePeriodDither::new();
        // 0 ppb -> nominal one-second period (tick_hz ticks); +1000 ppb at 1 MHz tick -> +1 tick.
        assert_eq!(d.next_ticks(1_000_000, 0), 1_000_000);
        assert_eq!(d.next_ticks(1_000_000, 1_000_000), 1_000_001);
    }

    #[test]
    fn naive_dither_resolves_sub_tick_on_average() {
        let mut d = NaivePeriodDither::new();
        // 500 ppb at a 1 MHz tick = half a tick/s: the fraction carries, alternating +0,+1,+0,+1.
        assert_eq!(d.next_ticks(1_000_000, 500_000), 1_000_000);
        assert_eq!(d.next_ticks(1_000_000, 500_000), 1_000_001);
        assert_eq!(d.next_ticks(1_000_000, 500_000), 1_000_000);
        assert_eq!(d.next_ticks(1_000_000, 500_000), 1_000_001);
    }

    #[test]
    fn naive_dither_negative_freq_shortens_period() {
        let mut d = NaivePeriodDither::new();
        // -1000 ppb at a 1 MHz tick -> -1 tick (a slow crystal needs a shorter wait).
        assert_eq!(d.next_ticks(1_000_000, -1_000_000), 999_999);
    }

    #[test]
    fn naive_dither_averages_below_one_tick_quantum() {
        // The dropped integer-µs alternative rounds the per-edge steering to whole ticks: at a 1 MHz
        // tick, 250 ppb rounds to 0 every edge (a 1000 ppb = 1-tick quantum), losing all sub-quantum
        // steering. The sigma-delta recovers it: 0.25 tick/s accumulates with no loss over many edges.
        let per_edge_whole = 1_000_000i64 * 250_000 / 1_000_000_000_000; // integer round = 0
        assert_eq!(per_edge_whole, 0);
        let mut d = NaivePeriodDither::new();
        let n = 4000i64;
        let mut excess = 0i64;
        for _ in 0..n {
            excess += d.next_ticks(1_000_000, 250_000) as i64 - 1_000_000;
        }
        assert_eq!(excess, n / 4); // 0.25 tick/s carried exactly
    }

    #[test]
    fn fold_phase_ns_is_invariant_to_whole_seconds() {
        // The sub-second phase is unchanged by adding/removing whole seconds from EITHER timestamp,
        // so the naive path's possible ±1 s mis-attribution does not move the internal phase metric.
        const SEC: i64 = 1_000_000_000;
        assert_eq!(fold_phase_ns(1_700_000_300, 1_700_000_000), 300);
        for ks in [-3i64, -1, 0, 2, 5] {
            for js in [-2i64, 0, 1, 4] {
                assert_eq!(
                    fold_phase_ns(1_700_000_300 + ks * SEC, 1_700_000_000 + js * SEC),
                    300
                );
            }
        }
    }

    #[test]
    fn fold_phase_ns_folds_to_half_second_window() {
        const SEC: i64 = 1_000_000_000;
        // 0.9 s lead folds to -0.1 s (closer to the next second).
        assert_eq!(fold_phase_ns(900_000_000, 0), -100_000_000);
        // exactly +½ s stays +½ s (closed upper bound); -½ s maps to +½ s (open lower bound).
        assert_eq!(fold_phase_ns(SEC / 2, 0), SEC / 2);
        assert_eq!(fold_phase_ns(-SEC / 2, 0), SEC / 2);
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
    fn edge_timeline_can_start_where_the_counter_did() {
        // X is left at zero before the state machine is enabled, so the counter runs down from
        // zero and the first captured value is the elapsed ticks negated. A timeline that starts
        // there can be lined up with any other state machine enabled by the same write.
        let mut t = PpsEdgeTimeline::from_counter_start(125_000_000);
        // 0.4 s of counting before the first edge: 25M ticks at 62.5M ticks/s.
        let first = t.observe(0u32.wrapping_sub(25_000_000));
        assert_eq!(first.edge_ns, 400_000_000, "first edge, measured from the start");
        let second = t.observe(0u32.wrapping_sub(25_000_000 + 62_500_000));
        assert_eq!(second.interval_ns, 1_000_000_000);
        assert_eq!(second.edge_ns, 1_400_000_000);
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
