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

use crate::PpsPolarity;
use embassy_rp::Peri;
use embassy_rp::pio::{
    Common, Config, Direction, Instance, LoadedProgram, Pin, PioPin, StateMachine,
};

/// Route `pin` into PIO so that [`crate::pps_capture_program`]'s rising-edge capture lands on the
/// edge that marks the second.
///
/// The program only ever watches for a rising edge, so an [`PpsPolarity::ActiveLow`] receiver has
/// to have its input inverted; an [`PpsPolarity::ActiveHigh`] one is already right and this clears
/// any inversion left over.
///
/// Call this **after** the pin has been handed to PIO. Assigning a pin to PIO rewrites the same
/// `GPIO_CTRL` register the inversion lives in, so an inversion set beforehand is silently dropped.
///
/// The inversion also applies to SIO, so `GPIO_IN` reads the inverted level afterwards. Anything
/// sampling the raw pin (a duty measurement, say) has to run before this or account for it.
pub fn set_capture_polarity(pin: usize, polarity: PpsPolarity) {
    use embassy_rp::pac::io::vals::Inover;

    let inover = match polarity {
        PpsPolarity::ActiveLow => Inover::INVERT,
        PpsPolarity::ActiveHigh => Inover::NORMAL,
    };
    embassy_rp::pac::IO_BANK0
        .gpio(pin)
        .ctrl()
        .modify(|w| w.set_inover(inover));
}

/// Start several state machines on one cycle, so the counters inside them agree.
///
/// Two free-running counters are only a shared timebase if they were started together, and
/// `SM_ENABLE` alone does not do that. Each state machine has its own clock divider running free,
/// so two enabled in the same write can still be counting on opposite phases of theirs — a
/// sub-tick error, but a permanent one, and the kind that shows up later as a fixed offset nobody
/// can account for. `CLKDIV_RESTART` in the same write is what the SDK's
/// `pio_enable_sm_mask_in_sync` exists for.
///
/// `mask` is a bitmask of state machine indices within `pio`; combine the `sm_mask()` of each
/// wrapper being started. Machines already running are left running, but their dividers *are*
/// restarted, so this is for bringing a set up together rather than adding one to a set that is
/// already timing something.
///
/// Counters compared against each other have to be in the same block. Nothing here can start a
/// state machine in PIO0 and one in PIO1 on the same cycle.
///
/// `pio` is the block's register file, e.g. `embassy_rp::pac::PIO0`. It is handed in because
/// embassy's `Instance` does not expose it.
pub fn start_in_sync(pio: embassy_rp::pac::pio::Pio, mask: u8) {
    pio.ctrl().modify(|w| {
        w.set_clkdiv_restart(mask);
        w.set_sm_enable(w.sm_enable() | mask);
    });
}

/// Point a state machine's `jmp` pin at a GPIO another block claimed.
///
/// Only one PIO block can *drive* a pin, and that is what `FUNCSEL` selects. Reading is not
/// driving: the input path belongs to the pad, and every block sees it. So a state machine can
/// watch a pin the other block is driving — which is exactly what timestamping an outgoing frame
/// at the pad means.
///
/// What embassy's API cannot express is that, because the type its `set_jmp_pin` takes is proof of
/// a claim this block has not made and must not make: claiming it would rewrite `FUNCSEL` and take
/// the pin away from whatever is driving it. All that is wanted is the pin *number*, so that is
/// what this writes.
///
/// The field lives in `EXECCTRL`, not `PINCTRL`, and `set_config` writes the whole of both — so
/// call this after the state machine's config has been set, or it is undone.
pub fn set_jmp_pin_unclaimed(pio: embassy_rp::pac::pio::Pio, sm: usize, gpio: u8) {
    pio.sm(sm).execctrl().modify(|w| w.set_jmp_pin(gpio));
}

/// A counter that timestamps a pin it does not own.
///
/// [`PpsCapture`] claims its pin, which is right for a 1PPS input and wrong for everything else on
/// a board where the pins are already spoken for. An outgoing frame is driven by a state machine in
/// the other block; a 1PPS output is driven by one in this block; both want timestamping at the
/// pad, and neither can be claimed a second time.
///
/// So this claims nothing. It runs the same counter program, watches a pin by number, and pushes
/// the counter on every rising edge. What it gives up is any check that the pin is configured at
/// all — an unconnected number reads as a pin that never moves.
///
/// **Every rising edge**, which for a 1PPS is one a second and for a frame is one per zero bit.
/// The program pushes without blocking, so the counter keeps running and the surplus is dropped
/// rather than stalling it; the first value in the FIFO after a quiet line is the frame's first
/// bit. See [`EventCapture::drain`].
pub struct EventCapture<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
    /// Kept alive only when this capture loaded it. Several state machines running the same
    /// program share one copy — a PIO block has 32 instructions, and three copies of an
    /// eleven-instruction counter do not fit in them. Holding it is bookkeeping rather than
    /// lifetime: what frees a block's instruction memory is the `Common` going away, and a
    /// firmware that keeps its state machines running must not let that happen anyway.
    #[allow(dead_code)]
    prog: Option<LoadedProgram<'d, PIO>>,
    /// Captures accounted for, read or discarded.
    ///
    /// Kept here rather than by the caller because the caller cannot see them all. This counter
    /// stops for [`crate::EVENT_CAPTURE_TOLL_TICKS`] every time it fires, and it fires on
    /// everything that crosses its pin — not only the frames a task went looking for. A count
    /// maintained beside the interesting ones drifts from the truth at the rate of the boring ones,
    /// and [`crate::ticks_between`] then corrects by the wrong amount.
    captures: u64,
    /// Whether a read ever emptied a full FIFO, so a capture may have been dropped before it.
    ///
    /// The program pushes without blocking, which is what keeps the counter running — and what
    /// makes a surplus silent. Four is the depth, so four in hand means there may have been a
    /// fifth, and from then on the count is a lower bound.
    may_have_missed: bool,
}

/// Depth of a state machine's RX FIFO when the FIFOs are not joined.
const RX_FIFO_DEPTH: u32 = 4;

impl<'d, PIO: Instance, const SM: usize> EventCapture<'d, PIO, SM> {
    /// Load `program` onto `sm` and point it at `gpio`, leaving it stopped.
    ///
    /// Stopped, because a counter is only a timebase together with the others it will be compared
    /// against — bring the set up with one [`start_in_sync`].
    ///
    /// `pio_regs` is this block's register file, e.g. `embassy_rp::pac::PIO0`. It has to be handed
    /// in because embassy's `Instance` does not expose it, and pointing a `jmp` pin at a GPIO this
    /// block has not claimed is exactly the thing embassy's API declines to express.
    pub fn new_stopped(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        pio_regs: embassy_rp::pac::pio::Pio,
        gpio: u8,
        program: &pio::Program<32>,
    ) -> Self {
        let prog = common.load_program(program);
        let mut me = Self::new_stopped_shared(sm, pio_regs, gpio, &prog);
        me.prog = Some(prog);
        me
    }

    /// How many times this counter has stopped to capture, as far as it has been told.
    ///
    /// Read it *before* taking the value it is about to be asked about: what
    /// [`crate::ticks_between`] wants is the tolls already paid when that value was pushed, and a
    /// capture's own toll comes after its push.
    pub fn captures(&self) -> u64 {
        self.captures
    }

    /// Whether the count is a lower bound rather than exact — see [`EventCapture::may_have_missed`
    /// documentation on the field]. Once true it stays true.
    pub fn missed_captures(&self) -> bool {
        self.may_have_missed
    }

    /// Like [`EventCapture::new_stopped`], but onto a program already in the block.
    ///
    /// A PIO block holds 32 instructions and every counter in a set runs the same program, so
    /// loading it once and pointing several state machines at it is not an optimisation — three
    /// copies of an eleven-instruction counter do not fit. The caller keeps the program alive.
    pub fn new_stopped_shared(
        mut sm: StateMachine<'d, PIO, SM>,
        pio_regs: embassy_rp::pac::pio::Pio,
        gpio: u8,
        prog: &LoadedProgram<'d, PIO>,
    ) -> Self {
        let mut cfg = Config::default();
        cfg.use_program(prog, &[]);
        sm.set_config(&cfg);
        // After `set_config`, which writes the whole of `EXECCTRL`.
        set_jmp_pin_unclaimed(pio_regs, SM, gpio);
        Self {
            sm,
            prog: None,
            captures: 0,
            may_have_missed: false,
        }
    }

    /// This capture's bit in its PIO's `CTRL` register, for [`start_in_sync`].
    pub const fn sm_mask() -> u8 {
        1 << SM
    }

    /// Hand [`crate::event_capture_program`] its blanking length, before starting.
    ///
    /// That program's first instruction is a blocking `pull`, so a state machine that is started
    /// without this never reaches its counting loop — and a counter that is not counting is not
    /// silent, it is wrong. Returns whether the word was taken.
    pub fn arm(&mut self, blank_counts: u32) -> bool {
        self.sm.tx().try_push(blank_counts)
    }

    /// The oldest counter value waiting, if any.
    pub fn try_read(&mut self) -> Option<u32> {
        let value = self.sm.rx().try_pull();
        if value.is_some() {
            self.captures += 1;
        }
        value
    }

    /// Await the next edge.
    ///
    /// Counted like every other way of taking a value out: what [`EventCapture::captures`] is for
    /// is the tolls the counter has paid, and it pays one whether the value was awaited, polled or
    /// thrown away. Leaving this one out would under-correct by one capture per awaited edge, which
    /// accumulates rather than cancels.
    pub async fn wait_edge(&mut self) -> u32 {
        let value = self.sm.rx().wait_pull().await;
        self.captures += 1;
        value
    }

    /// Throw away everything waiting, and say how much there was.
    ///
    /// A frame raises the watched pin hundreds of times. Only the first matters, so the rest are
    /// cleared before the next one is expected — and the count is worth having, because a FIFO that
    /// was already full when the frame started means the first value in it was not the first bit.
    pub fn drain(&mut self) -> u32 {
        let mut n = 0;
        while self.sm.rx().try_pull().is_some() {
            n += 1;
        }
        self.captures += n as u64;
        if n >= RX_FIFO_DEPTH {
            self.may_have_missed = true;
        }
        n
    }
}

/// PPS input capture on one state machine (see [`crate::pps_capture_program`]).
pub struct PpsCapture<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
    // Kept alive for the lifetime of the capture (RAII over the loaded program), unless the
    // program was loaded by someone else and is being shared.
    #[allow(dead_code)]
    prog: Option<LoadedProgram<'d, PIO>>,
    pin: Pin<'d, PIO>,
}

impl<'d, PIO: Instance, const SM: usize> PpsCapture<'d, PIO, SM> {
    /// Load the capture program onto `sm`, route `pps_pin` as its input/`jmp` pin, and enable it.
    pub fn new(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        pps_pin: Peri<'d, impl PioPin>,
    ) -> Self {
        Self::new_with_program(common, sm, pps_pin, &crate::pps_capture_program())
    }

    /// Like [`PpsCapture::new`] but with an explicit capture program, e.g.
    /// [`crate::pps_capture_program_wrap_balanced`]. Counters that are compared against each
    /// other should all run the same variant.
    pub fn new_with_program(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        pps_pin: Peri<'d, impl PioPin>,
        program: &pio::Program<32>,
    ) -> Self {
        let mut capture = Self::new_stopped(common, sm, pps_pin, program);
        capture.sm.set_enable(true);
        capture
    }

    /// Like [`PpsCapture::new_with_program`], but leaves the state machine stopped.
    ///
    /// For a set of counters that will be compared: configure them all, then bring them up with
    /// one [`start_in_sync`]. A capture enabled on its own is a capture whose counter has no fixed
    /// relationship to any other.
    pub fn new_stopped(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        pps_pin: Peri<'d, impl PioPin>,
        program: &pio::Program<32>,
    ) -> Self {
        let prog = common.load_program(program);
        let mut me = Self::new_stopped_shared(common, sm, pps_pin, &prog);
        me.prog = Some(prog);
        me
    }

    /// Like [`PpsCapture::new_stopped`], but onto a program already in the block — see
    /// [`EventCapture::new_stopped_shared`] for why that matters.
    pub fn new_stopped_shared(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        pps_pin: Peri<'d, impl PioPin>,
        prog: &LoadedProgram<'d, PIO>,
    ) -> Self {
        let pin = common.make_pio_pin(pps_pin);
        sm.set_pin_dirs(Direction::In, &[&pin]);
        let mut cfg = Config::default();
        cfg.use_program(prog, &[]);
        cfg.set_jmp_pin(&pin);
        sm.set_config(&cfg);
        // Same known zero as `new_stopped`: without it the count starts wherever `X` was, and a
        // counter with no origin cannot be lined up with the others this write will start.
        unsafe { sm.exec_instr(crate::zero_x_instruction()) };
        Self {
            sm,
            prog: None,
            pin,
        }
    }

    /// This capture's bit in its PIO's `CTRL` register, for [`start_in_sync`].
    pub const fn sm_mask() -> u8 {
        1 << SM
    }

    /// Hand [`crate::event_capture_program`] its blanking length, before starting.
    ///
    /// Only for that program; [`crate::pps_capture_program`] wants nothing pushed and would take
    /// this as a symbol count it has no use for. Returns whether the word was taken.
    pub fn arm(&mut self, blank_counts: u32) -> bool {
        self.sm.tx().try_push(blank_counts)
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

    /// Like [`TimedPpsCapture::new`] but with an explicit capture program, e.g.
    /// [`crate::pps_capture_program_wrap_balanced`].
    pub fn new_with_program(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        pps_pin: Peri<'d, impl PioPin>,
        clk_hz: u32,
        program: &pio::Program<32>,
    ) -> Self {
        Self {
            capture: PpsCapture::new_with_program(common, sm, pps_pin, program),
            timeline: crate::PpsEdgeTimeline::new(clk_hz),
        }
    }

    /// Like [`TimedPpsCapture::new_with_program`] but leaves the state machine stopped, so it can
    /// be brought up together with the other counters it will be compared against.
    /// `toll` is what one capture costs the counter, in ticks — zero for
    /// [`crate::pps_capture_program`], [`crate::EVENT_CAPTURE_TOLL_TICKS`] for
    /// [`crate::event_capture_program`]. Leaving it out reads every interval short by that much,
    /// which is a bias in the frequency estimate and not noise that averages away.
    pub fn new_stopped(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        pps_pin: Peri<'d, impl PioPin>,
        clk_hz: u32,
        toll: u64,
        program: &pio::Program<32>,
    ) -> Self {
        Self {
            capture: PpsCapture::new_stopped(common, sm, pps_pin, program),
            timeline: crate::PpsEdgeTimeline::from_counter_start_with_toll(clk_hz, toll),
        }
    }

    /// Like [`TimedPpsCapture::new_stopped`], but onto a program already in the block.
    pub fn new_stopped_shared(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        pps_pin: Peri<'d, impl PioPin>,
        clk_hz: u32,
        toll: u64,
        prog: &LoadedProgram<'d, PIO>,
    ) -> Self {
        Self {
            capture: PpsCapture::new_stopped_shared(common, sm, pps_pin, prog),
            timeline: crate::PpsEdgeTimeline::from_counter_start_with_toll(clk_hz, toll),
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

    /// This capture's bit in its PIO's `CTRL` register, for [`start_in_sync`].
    pub const fn sm_mask() -> u8 {
        1 << SM
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
    /// Load the output program onto `sm`, route `out_pin` as its `set` pin, push the `high_cycles`
    /// init word ([`crate::output_high_cycles`]) followed by `initial_period`, and enable it. Compute
    /// the period word with [`crate::output_period_cycles`].
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        out_pin: Peri<'d, impl PioPin>,
        high_cycles: u32,
        initial_period: u32,
    ) -> Self {
        let prog = common.load_program(&crate::pps_output_program());
        let pin = common.make_pio_pin(out_pin);
        sm.set_pin_dirs(Direction::Out, &[&pin]);
        let mut cfg = Config::default();
        cfg.use_program(&prog, &[]);
        cfg.set_set_pins(&[&pin]);
        sm.set_config(&cfg);
        let _ = sm.tx().try_push(high_cycles); // init: program stashes this as the high width
        let _ = sm.tx().try_push(initial_period);
        sm.set_enable(true);
        Self { sm, prog, pin }
    }

    /// Commit the next period word for the next output edge. Returns `false` if the TX FIFO was full
    /// (the push was dropped). NOTE: the program does **not** hold the last period on an empty FIFO —
    /// an empty `pull noblock` loads scratch `X` (a spent counter, garbage here), so a fresh period
    /// must be pushed every edge or the output drops a pulse (see [`crate::pps_output_program`]).
    /// Compute it with [`crate::output_period_cycles_ppb`].
    pub fn set_period(&mut self, period_word: u32) -> bool {
        self.sm.tx().try_push(period_word)
    }

    /// This output's bit in its PIO's `CTRL` register, for [`start_in_sync`].
    pub const fn sm_mask() -> u8 {
        1 << SM
    }

    /// Like [`PpsOutput::new`], but left stopped so it can be enabled alongside a capture.
    ///
    /// The output's phase is fixed by when its state machine started: the program raises the pin
    /// [`crate::OUTPUT_OVERHEAD_CYCLES`] cycles later and every edge after that is a period word
    /// this side counted out. Started on its own, that instant can only be read from a software
    /// clock, and whatever that read costs becomes the output's offset. Started by the same write
    /// as the capture, it is the capture counter's zero, and no clock is read at all.
    pub fn new_stopped(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        out_pin: Peri<'d, impl PioPin>,
        high_cycles: u32,
        initial_period: u32,
    ) -> Self {
        let prog = common.load_program(&crate::pps_output_program());
        let pin = common.make_pio_pin(out_pin);
        sm.set_pin_dirs(Direction::Out, &[&pin]);
        let mut cfg = Config::default();
        cfg.use_program(&prog, &[]);
        cfg.set_set_pins(&[&pin]);
        sm.set_config(&cfg);
        let _ = sm.tx().try_push(high_cycles); // init: program stashes this as the high width
        let _ = sm.tx().try_push(initial_period);
        Self { sm, prog, pin }
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
    high_cycles: u32,
}

impl<'d, PIO: Instance, const SM: usize> SteeredPpsOutput<'d, PIO, SM> {
    /// Load the output program (see [`PpsOutput::new`]) with a `pulse_ns`-wide high pulse
    /// ([`crate::output_high_cycles`]; ~100 ms is the common GPS-module/GPSDO convention, a few µs
    /// suits counters/scopes), starting at the nominal 1 Hz period for `clk_hz`
    /// ([`crate::output_period_cycles`]), and pair it with a fresh dither. The disciplined rising
    /// edge is unaffected by the pulse width.
    pub fn new(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        out_pin: Peri<'d, impl PioPin>,
        clk_hz: u32,
        pulse_ns: u32,
    ) -> Self {
        let high_cycles = crate::output_high_cycles(clk_hz, pulse_ns);
        Self {
            output: PpsOutput::new(
                common,
                sm,
                out_pin,
                high_cycles,
                crate::output_period_cycles(clk_hz, high_cycles),
            ),
            dither: crate::OutputPeriodDither::new(),
            clk_hz,
            high_cycles,
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
        let period =
            self.dither
                .next_period(self.clk_hz, freq_mppb, phase_corr_ns, self.high_cycles);
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
