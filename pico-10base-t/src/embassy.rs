//! `embassy-rp` backend (async). Enable with the `embassy-rp` feature.
//!
//! A thin wrapper that loads [`crate::phy::ser_10base_t_program`] onto one state machine and feeds
//! it symbol words by DMA. The caller owns the `Pio`'s `Common` and the `StateMachine` (from
//! `Pio::new(...)`), and passes the two TX pins and a DMA channel.
//!
//! The downstream binary selects the chip (e.g. `embassy-rp/rp2040`); this crate stays
//! chip-feature-agnostic, matching `rp-pps`.
//!
//! # Why DMA rather than pushing words
//!
//! The state machine retires one 32-bit word every 16 PIO cycles — 800 ns. Feeding it from the CPU
//! means hitting that deadline continuously for the whole frame (81.6 µs for an NTP packet, 1.2 ms
//! at MTU), which on this project's hardware would contend with the GPSDO's PIO capture and its
//! phase servo. Encoding the whole frame into RAM first and handing the buffer to DMA removes the
//! deadline entirely: the measured preparation cost is a fraction of a percent of the wire time
//! (see `tests/bench_host.rs`), so there is ample room to encode the next frame while this one is
//! still going out.

use embassy_rp::Peri;
use embassy_rp::dma::Channel as DmaChannel;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, LoadedProgram, Pin, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use fixed::FixedU32;
use fixed::types::extra::U8;

use crate::phy::{NLP_WORD, pio_clock_divider_bits, ser_10base_t_program};

/// 10BASE-T transmitter on one PIO state machine plus one DMA channel.
pub struct Tx10BaseT<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
    dma: DmaChannel<'d>,
    // Kept alive for as long as the transmitter runs (RAII over the loaded program and pins).
    #[allow(dead_code)]
    prog: LoadedProgram<'d, PIO>,
    #[allow(dead_code)]
    pins: (Pin<'d, PIO>, Pin<'d, PIO>),
}

impl<'d, PIO: Instance, const SM: usize> Tx10BaseT<'d, PIO, SM> {
    /// Load the serialiser onto `sm`, drive `tx_minus`/`tx_plus` as its side-set pair, and enable
    /// it. `clk_hz` is the system clock, used to divide down to [`PIO_CLOCK_HZ`].
    ///
    /// Pin order matters: the low side-set bit drives **TX−**, matching the symbol definitions in
    /// [`crate::phy`]. Swapping them inverts the differential pair, which still looks like a valid
    /// Manchester waveform on a scope and is undecodable by every receiver on the segment.
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        tx_minus: Peri<'d, impl PioPin>,
        tx_plus: Peri<'d, impl PioPin>,
        // Already constructed by the caller: `Channel::new` needs an interrupt binding, which
        // belongs with the firmware's `bind_interrupts!` rather than being demanded by a library.
        dma: DmaChannel<'d>,
        clk_hz: u32,
    ) -> Self {
        let prog = common.load_program(&ser_10base_t_program());
        let minus = common.make_pio_pin(tx_minus);
        let plus = common.make_pio_pin(tx_plus);
        sm.set_pin_dirs(Direction::Out, &[&minus, &plus]);

        let mut cfg = Config::default();
        cfg.use_program(&prog, &[&minus, &plus]);
        // Symbols are consumed LSB-first, one 32-bit word at a time, refilled automatically.
        cfg.shift_out = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Right,
            auto_fill: true,
        };
        // Nothing is ever received, so give the RX FIFO's depth to TX: 8 words of slack instead
        // of 4 between a DMA burst and the state machine draining it.
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.clock_divider = FixedU32::<U8>::from_bits(pio_clock_divider_bits(clk_hz));
        sm.set_config(&cfg);
        sm.set_enable(true);

        Self {
            sm,
            dma,
            prog,
            pins: (minus, plus),
        }
    }

    /// Transmit one already-encoded frame (see [`crate::phy::encode_frame`]).
    ///
    /// Returns once the DMA has handed every word to the FIFO. The state machine is still draining
    /// the last few words at that point — the FIFO holds up to 8, i.e. 6.4 µs.
    pub async fn send(&mut self, symbols: &[u32]) {
        self.sm.tx().dma_push(&mut self.dma, symbols, false).await;
    }

    /// Emit one normal link pulse. With no traffic, sending these every
    /// [`crate::phy::NLP_INTERVAL_US`] is what keeps the far end's link up.
    ///
    /// Non-blocking and lossy by design: if the FIFO is full a frame is already going out, which
    /// serves the same purpose as a link pulse, so dropping this one is correct.
    pub fn link_pulse(&mut self) -> bool {
        self.sm.tx().try_push(NLP_WORD)
    }

    /// Whether the state machine has drained everything handed to it. `&mut` because reaching the
    /// FIFO goes through the state machine's exclusive handle.
    pub fn is_idle(&mut self) -> bool {
        self.sm.tx().empty()
    }
}

// No tests here: this file needs the target to compile at all. Everything that can be decided
// without a chip — the symbol encoding, the program's shape, the clock divider — lives in
// `crate::phy` and is tested there on the host.
