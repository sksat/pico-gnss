//! `embassy-rp` backend (async). Enable with the `embassy-rp` feature.
//!
//! Thin wrappers that load the [crate-level](crate) PIO programs and `.await` the RX FIFO / push
//! the TX FIFO. The caller owns the `Pio`'s `Common` and the `StateMachine` (from `Pio::new(...)`)
//! and passes the pin peripheral; each wrapper keeps its loaded program and pin alive for as long
//! as it runs.
//!
//! The downstream binary selects the chip (e.g. `embassy-rp/rp2040`); this crate is
//! chip-feature-agnostic. There is no shared trait with the `rp2040-hal` backend — the two HALs
//! differ enough (async vs blocking, ownership) that each is its own small concrete type.

use embassy_rp::Peri;
use embassy_rp::pio::{
    Common, Config, Direction, Instance, LoadedProgram, Pin, PioPin, StateMachine,
};

/// PPS input capture on one state machine (see [`crate::pps_capture_program`]).
pub struct PpsCapture<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
    // Kept alive for the lifetime of the capture (RAII over the loaded program).
    #[allow(dead_code)]
    prog: LoadedProgram<'d, PIO>,
    pin: Pin<'d, PIO>,
}

impl<'d, PIO: Instance, const SM: usize> PpsCapture<'d, PIO, SM> {
    /// Load the capture program onto `sm`, route `pps_pin` as its input/`jmp` pin, and enable it.
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        pps_pin: Peri<'d, impl PioPin>,
    ) -> Self {
        let prog = common.load_program(&crate::pps_capture_program());
        let pin = common.make_pio_pin(pps_pin);
        sm.set_pin_dirs(Direction::In, &[&pin]);
        let mut cfg = Config::default();
        cfg.use_program(&prog, &[]);
        cfg.set_jmp_pin(&pin);
        sm.set_config(&cfg);
        sm.set_enable(true);
        Self { sm, prog, pin }
    }

    /// Await the next rising edge; returns the raw down-counter value. Feed consecutive values to
    /// [`crate::interval_ns`] / [`crate::interval_ticks`].
    pub async fn wait_edge(&mut self) -> u32 {
        self.sm.rx().wait_pull().await
    }

    /// Non-blocking read of the raw down-counter value at the latest captured edge, if any (the
    /// HAL-generic equivalent is [`crate::PpsCaptureRead::try_read`]).
    pub fn try_read(&mut self) -> Option<u32> {
        self.sm.rx().try_pull()
    }

    /// The pin routed as this SM's `jmp` pin (the captured PPS input). Exposed so another state
    /// machine can watch the same physical pin — embassy-rp's `Config::set_jmp_pin` needs the
    /// `&Pin`, and a given pin can only be made once. (Used e.g. by a loopback measurement that
    /// must capture the same edge on a second SM.)
    pub fn jmp_pin(&self) -> &Pin<'d, PIO> {
        &self.pin
    }
}

/// [`PpsCapture`] paired with a [`PpsEdgeTimeline`](crate::PpsEdgeTimeline): each awaited edge comes
/// back already timed (interval + running timeline). The easy tier over the capture primitives.
///
/// Reach for the fine tier ([`PpsCapture`] + [`crate::PpsEdgeTimeline`] held separately) when you
/// need the raw counter at a different point than the timed edge — e.g. sharing the same physical
/// edge with another state machine for loopback phase, where the raw value is read and published
/// before any timeline bookkeeping.
pub struct TimedPpsCapture<'d, PIO: Instance, const SM: usize> {
    capture: PpsCapture<'d, PIO, SM>,
    timeline: crate::PpsEdgeTimeline,
}

impl<'d, PIO: Instance, const SM: usize> TimedPpsCapture<'d, PIO, SM> {
    /// Load the capture program (see [`PpsCapture::new`]) and pair it with a fresh timeline for
    /// `clk_hz`.
    pub fn new(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        pps_pin: Peri<'d, impl PioPin>,
        clk_hz: u32,
    ) -> Self {
        Self {
            capture: PpsCapture::new(common, sm, pps_pin),
            timeline: crate::PpsEdgeTimeline::new(clk_hz),
        }
    }

    /// Await the next rising edge and return it timed against the previous one
    /// ([`PpsCapture::wait_edge`] then [`crate::PpsEdgeTimeline::observe`]).
    pub async fn next_edge(&mut self) -> crate::TimedEdge {
        let raw = self.capture.wait_edge().await;
        self.timeline.observe(raw)
    }

    /// Borrow the underlying raw capture (the fine tier) — e.g. for [`PpsCapture::jmp_pin`].
    pub fn capture(&self) -> &PpsCapture<'d, PIO, SM> {
        &self.capture
    }

    /// Mutably borrow the underlying raw capture (e.g. [`PpsCapture::try_read`]).
    pub fn capture_mut(&mut self) -> &mut PpsCapture<'d, PIO, SM> {
        &mut self.capture
    }
}

/// Steerable 1PPS output on one state machine (see [`crate::pps_output_program`]).
pub struct PpsOutput<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
    #[allow(dead_code)]
    prog: LoadedProgram<'d, PIO>,
    #[allow(dead_code)]
    pin: Pin<'d, PIO>,
}

impl<'d, PIO: Instance, const SM: usize> PpsOutput<'d, PIO, SM> {
    /// Load the output program onto `sm`, route `out_pin` as its `set` pin, push `initial_period`,
    /// and enable it. Compute the period word with [`crate::output_period_cycles`].
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        out_pin: Peri<'d, impl PioPin>,
        initial_period: u32,
    ) -> Self {
        let prog = common.load_program(&crate::pps_output_program());
        let pin = common.make_pio_pin(out_pin);
        sm.set_pin_dirs(Direction::Out, &[&pin]);
        let mut cfg = Config::default();
        cfg.use_program(&prog, &[]);
        cfg.set_set_pins(&[&pin]);
        sm.set_config(&cfg);
        let _ = sm.tx().try_push(initial_period);
        sm.set_enable(true);
        Self { sm, prog, pin }
    }

    /// Commit the next period word (the program holds the previous one until then). Returns `false`
    /// if the TX FIFO was full. Compute it with [`crate::output_period_cycles_ppb`].
    pub fn set_period(&mut self, period_word: u32) -> bool {
        self.sm.tx().try_push(period_word)
    }
}

/// [`PpsOutput`] paired with an [`OutputPeriodDither`](crate::OutputPeriodDither): steer the 1PPS by
/// a total frequency offset + an immediate phase nudge in one call. The easy tier over the output
/// primitives — it owns the sigma-delta accumulator and the system clock, so the caller passes only
/// milli-ppb and a phase correction instead of hand-rolling the period-word math each edge.
///
/// Use the fine tier ([`PpsOutput`] + [`crate::OutputPeriodDither`]) if you need the period word for
/// something else, or a different dither policy. Implements [`crate::PpsSteer`].
pub struct SteeredPpsOutput<'d, PIO: Instance, const SM: usize> {
    output: PpsOutput<'d, PIO, SM>,
    dither: crate::OutputPeriodDither,
    clk_hz: u32,
}

impl<'d, PIO: Instance, const SM: usize> SteeredPpsOutput<'d, PIO, SM> {
    /// Load the output program (see [`PpsOutput::new`]) starting at the nominal 1 Hz period for
    /// `clk_hz` ([`crate::output_period_cycles`]), and pair it with a fresh dither.
    pub fn new(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        out_pin: Peri<'d, impl PioPin>,
        clk_hz: u32,
    ) -> Self {
        Self {
            output: PpsOutput::new(common, sm, out_pin, crate::output_period_cycles(clk_hz)),
            dither: crate::OutputPeriodDither::new(),
            clk_hz,
        }
    }

    /// Borrow the underlying raw output (the fine tier).
    pub fn output(&self) -> &PpsOutput<'d, PIO, SM> {
        &self.output
    }

    /// Mutably borrow the underlying raw output (e.g. a one-off [`PpsOutput::set_period`]).
    pub fn output_mut(&mut self) -> &mut PpsOutput<'d, PIO, SM> {
        &mut self.output
    }
}

impl<'d, PIO: Instance, const SM: usize> crate::PpsSteer for SteeredPpsOutput<'d, PIO, SM> {
    fn set_next_period(&mut self, freq_mppb: i64, phase_corr_ns: i64) -> u32 {
        let period = self
            .dither
            .next_period(self.clk_hz, freq_mppb, phase_corr_ns);
        let _ = self.output.set_period(period);
        period
    }
}

impl<'d, PIO: Instance, const SM: usize> crate::PpsCaptureRead for PpsCapture<'d, PIO, SM> {
    fn try_read(&mut self) -> Option<u32> {
        self.sm.rx().try_pull()
    }
}

impl<'d, PIO: Instance, const SM: usize> crate::PpsPeriodSet for PpsOutput<'d, PIO, SM> {
    fn set_period(&mut self, period_word: u32) -> bool {
        self.sm.tx().try_push(period_word)
    }
}

/// Easy-tier runner tasks (`gnssdo` feature): drive a shared [`PpsGpsdo`](crate::PpsGpsdo) from the
/// capture and the receiver's NMEA, so the app only spawns these and reads disciplined UTC. See the
/// `gpsdo_runner` example (vs the `gpsdo` example, which calls the `PpsGpsdo` methods by hand).
#[cfg(feature = "gnssdo")]
mod runner {
    use super::{Instance, TimedPpsCapture};
    use core::cell::RefCell;
    use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
    use embassy_sync::blocking_mutex::raw::RawMutex;
    use embedded_io_async::Read;

    /// Run the PPS capture loop, feeding each timed edge (frequency discipline + epoch-pairing
    /// record) into the shared `clock`. Never returns — spawn it from your own `#[task]`. `query_ns`
    /// supplies the query-timebase value at each edge (e.g. `|| Instant::now().as_micros() * 1000`).
    pub async fn run_capture<M, PIO, const SM: usize>(
        mut capture: TimedPpsCapture<'_, PIO, SM>,
        clock: &BlockingMutex<M, RefCell<crate::PpsGpsdo>>,
        query_ns: impl Fn() -> u64,
    ) -> !
    where
        M: RawMutex,
        PIO: Instance,
    {
        loop {
            let edge = capture.next_edge().await;
            clock.lock(|g| {
                g.borrow_mut().on_pps_edge(edge, query_ns());
            });
        }
    }

    /// Run the NMEA ingest loop: read framed sentences from `nmea_rx` and feed each to the shared
    /// `clock` (an RMC paired with a fresh PPS edge establishes the UTC epoch). Never returns —
    /// spawn it from your own `#[task]`. Framing is handled internally with
    /// [`NmeaLineAssembler`](crate::NmeaLineAssembler).
    pub async fn run_nmea<M, R>(
        mut nmea_rx: R,
        clock: &BlockingMutex<M, RefCell<crate::PpsGpsdo>>,
    ) -> !
    where
        M: RawMutex,
        R: Read,
    {
        let mut assembler = crate::NmeaLineAssembler::new();
        let mut buf = [0u8; 64];
        loop {
            let Ok(n) = nmea_rx.read(&mut buf).await else {
                continue; // framing/overrun is common at start-up; resync on the next '$'
            };
            for &b in &buf[..n] {
                if let Some(sentence) = assembler.push(b) {
                    if let Ok(s) = core::str::from_utf8(sentence) {
                        clock.lock(|g| {
                            g.borrow_mut().feed_nmea(s);
                        });
                    }
                }
            }
        }
    }
}

#[cfg(feature = "gnssdo")]
pub use runner::{run_capture, run_nmea};
