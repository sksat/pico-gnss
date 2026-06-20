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
        initial_period: u32,
    ) -> Self {
        let installed = pio
            .install(&crate::pps_output_program())
            .expect("PIO instruction memory full");
        let (mut sm, _rx, mut tx) = PIOBuilder::from_installed_program(installed)
            .set_pins(out_gpio, 1)
            .build(sm);
        sm.set_pindirs([(out_gpio, PinDir::Output)]);
        tx.write(initial_period);
        Self { sm: sm.start(), tx }
    }

    /// Commit the next period word (the program holds the previous one until this is read).
    /// Returns `false` if the TX FIFO was full. Compute it with [`crate::output_period_cycles_ppb`].
    pub fn set_period(&mut self, period_word: u32) -> bool {
        self.tx.write(period_word)
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
