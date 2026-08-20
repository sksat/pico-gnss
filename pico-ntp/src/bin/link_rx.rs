//! Read what the other Pico is sending, and say what arrived.
//!
//! The second board in the pair. Its only job here is to show that the link carries frames: it
//! captures what follows every departure from idle, decodes it, checks the FCS, and reports.
//!
//! ```text
//!   server GP16 (TX−) ──► GP18   PIO0 SM0 + DMA ──► FrameDecoder ──► defmt
//!   server GP17 (TX+) ──► GP19
//! ```
//!
//! Two pins, not one. The line has three states — idle and the two differential polarities — and
//! reading only one of them cannot tell idle from a run of zeros, which is where a frame ends.
//!
//! Nothing is transmitted from here. That comes with the NTP client, once this says the bytes
//! survive the wire.

#![no_std]
#![no_main]

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::{Duration, Instant};

use pico_10base_t::embassy::Rx10BaseT;
use pico_10base_t::rx::{decode_frame, symbols_of};

use defmt_rtt as _;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>;
});

/// Words to take once the line moves. An NTP frame is 94 bytes, which with preamble and TP_IDL is
/// 1638 symbols; at two samples each that is 205 words. The rest of the capture is idle, and the
/// decoder ignores it.
const CAPTURE_WORDS: usize = 256;

/// Room for the largest frame this link is expected to carry.
const MAX_FRAME: usize = 256;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("link_rx: watching GP18 (TX-) and GP19 (TX+)");

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    let dma = embassy_rp::dma::Channel::new(p.DMA_CH0, Irqs);
    let mut rx = Rx10BaseT::new(&mut common, sm0, p.PIN_18, p.PIN_19, dma, clk_sys_freq());

    let mut words = [0u32; CAPTURE_WORDS];
    let mut frame = [0u8; MAX_FRAME];
    let mut seen: u32 = 0;
    let mut good: u32 = 0;
    let mut last_report = Instant::now();

    loop {
        rx.capture(&mut words).await;
        seen = seen.wrapping_add(1);

        let Some(len) = decode_frame(&words, &mut frame) else {
            // Not a frame, or not one that survived its FCS. Say what did arrive, so the two cases
            // can be told apart: a link pulse is a handful of samples, a frame is thousands.
            let nonidle: u32 = words
                .iter()
                .map(|w| symbols_of(*w).into_iter().filter(|s| *s != 0).count() as u32)
                .sum();
            warn!("no frame: nonidle={} head={:08x}", nonidle, words[0]);
            continue;
        };

        good = good.wrapping_add(1);
        // The first few in full, so the host can turn them into a pcap and let Wireshark judge.
        if good <= 3 {
            info!("LINKRX n={} bytes={=[u8]:02x}", good, &frame[..len]);
        }
        if last_report.elapsed() >= Duration::from_secs(10) {
            info!(
                "LINKRX captures={} good={} last_len={}",
                seen,
                good,
                len + 4
            );
            last_report = Instant::now();
        }
    }
}
