#![no_std]
#![no_main]

//! On-target benchmark: what transmit rate does `pico-10base-t` actually reach on an RP2040?
//!
//! Not built by a normal build (`bench` feature required). On hardware:
//!
//! ```text
//! cd pico-ntp && cargo run --release --features bench --bin bench_tx
//! ```
//!
//! # What it answers
//!
//! The host benchmark (`pico-10base-t/tests/bench_host.rs`) says preparation costs ~0.37% of the
//! time a frame occupies the wire, but that is an x86 number and the extrapolation to a Cortex-M0+
//! spans 10–40%. This measures it, which is the only way to know whether the CPU or the wire is the
//! limit on the part that actually ships.
//!
//! It reports, per frame size:
//!
//! - **prepare** — building the frame (headers, checksums, FCS) plus Manchester-encoding it.
//! - **send** — handing the symbol buffer to DMA and waiting for it to drain.
//! - **wire** — how long the frame occupies 10BASE-T, computed from its length. If prepare is well
//!   under this, the link can be kept busy by encoding the next frame while this one goes out.
//! - **effective rate** — UDP payload bits divided by the whole prepare+send cycle, i.e. what a
//!   caller would actually get back-to-back, including the interframe gap.
//!
//! Nothing is disciplined here and no GNSS is needed: the frames carry filler. This measures the
//! transmit path alone, deliberately.

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::{Instant, Timer};

use pico_10base_t::embassy::Tx10BaseT;
use pico_10base_t::frame::{Ipv4Addr, MacAddr, UdpFrameSpec, build_udp_frame, frame_len};
use pico_10base_t::phy::{encode_frame, encoded_words};

bind_interrupts!(struct Irqs {
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>;
});

/// Largest UDP payload that fits a 1500-byte IPv4 MTU.
const MAX_PAYLOAD: usize = 1472;
const MAX_FRAME: usize = frame_len(MAX_PAYLOAD);
const MAX_WORDS: usize = encoded_words(MAX_FRAME);

/// Preamble + SFD, on the wire ahead of every frame.
const PREAMBLE_LEN: usize = 8;
/// Interframe gap: 96 bit times.
const IFG_LEN: usize = 12;
/// Frames per size. Enough to average out timer granularity, few enough to finish quickly.
const ITERS: u32 = 200;

static mut FRAME: [u8; MAX_FRAME] = [0; MAX_FRAME];
static mut SYMBOLS: [u32; MAX_WORDS] = [0; MAX_WORDS];

fn spec(payload: &[u8]) -> UdpFrameSpec<'_> {
    UdpFrameSpec {
        src_mac: MacAddr([0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEE]),
        dst_mac: MacAddr::BROADCAST,
        src_ip: Ipv4Addr::new(192, 168, 0, 200),
        dst_ip: Ipv4Addr::BROADCAST,
        src_port: 123,
        dst_port: 123,
        ip_id: 0,
        ttl: 1,
        payload,
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    Timer::after_millis(800).await; // let the probe attach before the first line

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO1, Irqs);
    let dma = embassy_rp::dma::Channel::new(p.DMA_CH0, Irqs);
    let mut tx = Tx10BaseT::new(&mut common, sm0, p.PIN_16, p.PIN_17, dma, clk_sys_freq());

    let filler = [0xA5u8; MAX_PAYLOAD];
    info!(
        "bench_tx: {} frames per size, RP2040 @ {} Hz",
        ITERS,
        clk_sys_freq()
    );
    info!("payload frame prepare_us send_us wire_us cpu_pct eff_kbps");

    for &n in &[1usize, 48, 256, 512, 1024, MAX_PAYLOAD] {
        let frame_len_n = frame_len(n);
        // SAFETY: single-threaded benchmark; these buffers have no other user.
        let frame = unsafe { &mut *core::ptr::addr_of_mut!(FRAME) };
        let symbols = unsafe { &mut *core::ptr::addr_of_mut!(SYMBOLS) };

        // Prepare (build + encode), timed on its own so it can be compared against wire time.
        let t0 = Instant::now();
        let mut words = 0;
        for _ in 0..ITERS {
            let len = build_udp_frame(&spec(&filler[..n]), frame).unwrap();
            words = encode_frame(&frame[..len], symbols).unwrap();
        }
        let prepare_ns = t0.elapsed().as_micros() * 1000 / ITERS as u64;

        // Send, which is DMA plus the state machine draining behind it.
        let t1 = Instant::now();
        for _ in 0..ITERS {
            tx.send(&symbols[..words]).await;
            // IEEE 802.3 wants 96 bit times between frames. `send` returns once the DMA is queued
            // and the only separator the symbol stream carries is the 800 ns TP_IDL word, so
            // without this the loop puts frames on the pair closer together than the standard
            // allows — and then reports a rate no compliant transmitter could reach.
            Timer::after_nanos(IFG_LEN as u64 * 8 * 100).await;
        }
        let send_ns = t1.elapsed().as_micros() * 1000 / ITERS as u64;

        // Wire occupancy of the frame itself: 100 ns per bit at 10 Mbit/s.
        let wire_ns = (frame_len_n + PREAMBLE_LEN) as u64 * 8 * 100;
        // What a caller gets back-to-back: payload bits over the whole cycle. The interframe gap
        // is already inside `send_ns`, since the loop above waits it out.
        let cycle_ns = prepare_ns + send_ns;
        let eff_kbps = (n as u64 * 8 * 1_000_000) / cycle_ns.max(1);

        info!(
            "{} {} {} {} {} {} {}",
            n,
            frame_len_n,
            prepare_ns / 1000,
            send_ns / 1000,
            wire_ns / 1000,
            prepare_ns * 100 / wire_ns.max(1),
            eff_kbps
        );
    }

    info!("bench_tx: done");
    loop {
        Timer::after_secs(60).await;
    }
}
