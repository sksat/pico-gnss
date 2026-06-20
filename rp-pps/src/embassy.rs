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
