//! `rp2040-hal` backend (blocking / IRQ-driven). Enable with the `rp2040-hal` feature.
//!
//! Thin wrappers that load the [crate-level](crate) PIO programs and expose non-blocking FIFO
//! access. Following rp2040-hal's model, the caller owns the `PIO` and the `UninitStateMachine`
//! (from `pac.PIOx.split(&mut resets)`) and the GPIOs — each PPS pin must already be switched into
//! PIO function (`pin.into_function::<FunctionPio0>()`) and kept alive; only its **GPIO number** is
//! passed here.
//!
//! There is no shared trait with the `embassy-rp` backend: the two HALs have different setup and
//! ownership models, so each backend is its own small concrete type over the same core programs.

use rp2040_hal::pio::{
    PIO, PIOBuilder, PIOExt, PinDir, Running, Rx, StateMachine, StateMachineIndex, Tx,
    UninitStateMachine,
};

/// PPS input capture on one state machine (see [`crate::pps_capture_program`]).
pub struct PpsCapture<P: PIOExt, SM: StateMachineIndex> {
    // Held to keep the SM running (and as the handle for a future stop()); never read directly.
    #[allow(dead_code)]
    sm: StateMachine<(P, SM), Running>,
    rx: Rx<(P, SM)>,
}

impl<P: PIOExt, SM: StateMachineIndex> PpsCapture<P, SM> {
    /// Install the capture program and start the SM. `pps_gpio` is the GPIO number of the PPS
    /// input (already in PIO function).
    ///
    /// # Panics
    /// If the program does not fit in the PIO's instruction memory.
    pub fn new(pio: &mut PIO<P>, sm: UninitStateMachine<(P, SM)>, pps_gpio: u8) -> Self {
        let installed = pio
            .install(&crate::pps_capture_program())
            .expect("PIO instruction memory full");
        let (mut sm, rx, _tx) = PIOBuilder::from_installed_program(installed)
            .jmp_pin(pps_gpio)
            .build(sm);
        sm.set_pindirs([(pps_gpio, PinDir::Input)]);
        Self { sm: sm.start(), rx }
    }

    /// Non-blocking read of the raw down-counter value at the latest captured edge, if any. Feed
    /// consecutive values to [`crate::interval_ns`] / [`crate::interval_ticks`] (the HAL-generic
    /// equivalent is [`crate::PpsCaptureRead::try_read`]).
    pub fn try_read(&mut self) -> Option<u32> {
        self.rx.read()
    }
}

/// [`PpsCapture`] paired with a [`PpsEdgeTimeline`](crate::PpsEdgeTimeline): each captured edge comes
/// back already timed (interval + running timeline). The easy tier over the capture primitives.
///
/// Reach for the fine tier ([`PpsCapture`] + [`crate::PpsEdgeTimeline`] held separately) when you
/// need the raw counter at a different point than the timed edge (e.g. sharing the edge with another
/// state machine for loopback phase).
pub struct TimedPpsCapture<P: PIOExt, SM: StateMachineIndex> {
    capture: PpsCapture<P, SM>,
    timeline: crate::PpsEdgeTimeline,
}

impl<P: PIOExt, SM: StateMachineIndex> TimedPpsCapture<P, SM> {
    /// Install the capture program (see [`PpsCapture::new`]) and pair it with a fresh timeline for
    /// `clk_hz`.
    ///
    /// # Panics
    /// If the program does not fit in the PIO's instruction memory.
    pub fn new(
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        pps_gpio: u8,
        clk_hz: u32,
    ) -> Self {
        Self {
            capture: PpsCapture::new(pio, sm, pps_gpio),
            timeline: crate::PpsEdgeTimeline::new(clk_hz),
        }
    }

    /// Non-blocking: if an edge was captured since the last call, return it timed against the
    /// previous one ([`PpsCapture::try_read`] then [`crate::PpsEdgeTimeline::observe`]); else `None`.
    pub fn try_timed_edge(&mut self) -> Option<crate::TimedEdge> {
        self.capture
            .try_read()
            .map(|raw| self.timeline.observe(raw))
    }

    /// Borrow the underlying raw capture (the fine tier).
    pub fn capture(&self) -> &PpsCapture<P, SM> {
        &self.capture
    }

    /// Mutably borrow the underlying raw capture (e.g. [`PpsCapture::try_read`]).
    pub fn capture_mut(&mut self) -> &mut PpsCapture<P, SM> {
        &mut self.capture
    }
}

/// Steerable 1PPS output on one state machine (see [`crate::pps_output_program`]).
pub struct PpsOutput<P: PIOExt, SM: StateMachineIndex> {
    #[allow(dead_code)]
    sm: StateMachine<(P, SM), Running>,
    tx: Tx<(P, SM)>,
}

impl<P: PIOExt, SM: StateMachineIndex> PpsOutput<P, SM> {
    /// Install the output program, push the initial period, and start the SM. `out_gpio` is the
    /// GPIO number of the PPS output (already in PIO function); `initial_period` is the first
    /// period word (e.g. [`crate::output_period_cycles`]).
    ///
    /// # Panics
    /// If the program does not fit in the PIO's instruction memory.
    pub fn new(
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        out_gpio: u8,
        high_cycles: u32,
        initial_period: u32,
    ) -> Self {
        let installed = pio
            .install(&crate::pps_output_program())
            .expect("PIO instruction memory full");
        let (mut sm, _rx, mut tx) = PIOBuilder::from_installed_program(installed)
            .set_pins(out_gpio, 1)
            .build(sm);
        sm.set_pindirs([(out_gpio, PinDir::Output)]);
        tx.write(high_cycles); // init: program stashes this as the high width
        tx.write(initial_period);
        Self { sm: sm.start(), tx }
    }

    /// Commit the next period word (the program holds the previous one until this is read).
    /// Returns `false` if the TX FIFO was full. Compute it with [`crate::output_period_cycles_ppb`].
    pub fn set_period(&mut self, period_word: u32) -> bool {
        self.tx.write(period_word)
    }
}

/// [`PpsOutput`] paired with an [`OutputPeriodDither`](crate::OutputPeriodDither): steer the 1PPS by
/// a total frequency offset + an immediate phase nudge in one call. The easy tier over the output
/// primitives — it owns the sigma-delta accumulator and the system clock. Implements
/// [`crate::PpsSteer`]. Use the fine tier ([`PpsOutput`] + [`crate::OutputPeriodDither`]) for a
/// different dither policy or to reuse the period word.
pub struct SteeredPpsOutput<P: PIOExt, SM: StateMachineIndex> {
    output: PpsOutput<P, SM>,
    dither: crate::OutputPeriodDither,
    clk_hz: u32,
    high_cycles: u32,
}

impl<P: PIOExt, SM: StateMachineIndex> SteeredPpsOutput<P, SM> {
    /// Install the output program (see [`PpsOutput::new`]) with a `pulse_ns`-wide high pulse
    /// ([`crate::output_high_cycles`]; ~100 ms is the common GPS-module/GPSDO convention, a few µs
    /// suits counters/scopes), starting at the nominal 1 Hz period for `clk_hz`
    /// ([`crate::output_period_cycles`]), and pair it with a fresh dither. The disciplined rising
    /// edge is unaffected by the pulse width.
    ///
    /// # Panics
    /// If the program does not fit in the PIO's instruction memory.
    pub fn new(
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        out_gpio: u8,
        clk_hz: u32,
        pulse_ns: u32,
    ) -> Self {
        let high_cycles = crate::output_high_cycles(clk_hz, pulse_ns);
        Self {
            output: PpsOutput::new(
                pio,
                sm,
                out_gpio,
                high_cycles,
                crate::output_period_cycles(clk_hz, high_cycles),
            ),
            dither: crate::OutputPeriodDither::new(),
            clk_hz,
            high_cycles,
        }
    }

    /// Borrow the underlying raw output (the fine tier).
    pub fn output(&self) -> &PpsOutput<P, SM> {
        &self.output
    }

    /// Mutably borrow the underlying raw output (e.g. a one-off [`PpsOutput::set_period`]).
    pub fn output_mut(&mut self) -> &mut PpsOutput<P, SM> {
        &mut self.output
    }
}

impl<P: PIOExt, SM: StateMachineIndex> crate::PpsSteer for SteeredPpsOutput<P, SM> {
    fn set_next_period(&mut self, freq_mppb: i64, phase_corr_ns: i64) -> u32 {
        let period =
            self.dither
                .next_period(self.clk_hz, freq_mppb, phase_corr_ns, self.high_cycles);
        let _ = self.output.set_period(period);
        period
    }
}

impl<P: PIOExt, SM: StateMachineIndex> crate::PpsCaptureRead for PpsCapture<P, SM> {
    fn try_read(&mut self) -> Option<u32> {
        self.rx.read()
    }
}

impl<P: PIOExt, SM: StateMachineIndex> crate::PpsPeriodSet for PpsOutput<P, SM> {
    fn set_period(&mut self, period_word: u32) -> bool {
        self.tx.write(period_word)
    }
}
