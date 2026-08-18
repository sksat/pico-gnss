#![no_std]
#![no_main]

//! **Stratum-1 NTP broadcast server on an RP2040.**
//!
//! A GNSS 1PPS-disciplined clock serving time over 10BASE-T Ethernet driven straight from two GPIO
//! pins and three resistors — no PHY chip, no MAC.
//!
//! ```text
//! GNSS module ──NMEA──> UART0 RX (GP1)  ──┐
//!             ──1PPS──> GP2 (PIO0 SM0)  ──┴─> PpsGpsdo ──> disciplined UTC
//!                                                            │
//!                                    ntp-refclock ───────────┘  (48-byte NTP packet)
//!                                          │
//!                                    pico-10base-t            (Ethernet/IPv4/UDP + Manchester)
//!                                          │
//!                                    PIO1 SM0 + DMA ──> GP16 (TX−) / GP17 (TX+)
//! ```
//!
//! # Wiring
//!
//! | Signal | Pin |
//! |---|---|
//! | GNSS NMEA TX → UART0 RX | GP1 (9600 baud) |
//! | GNSS 1PPS | GP2 |
//! | Ethernet TX− | GP16 |
//! | Ethernet TX+ | GP17 |
//!
//! 2 × 47 Ω in series with the TX pins and 1 × 470 Ω across the pair, into an RJ45's pins 1 and 2.
//! A pulse transformer is strongly recommended: without one there is no galvanic isolation between
//! the Pico and whatever it is plugged into.
//!
//! PIO0 belongs to the PPS capture and PIO1 to the Ethernet serialiser, so the two never contend.
//!
//! # Receiving this
//!
//! Broadcast mode (RFC 5905 mode 5) is one-way, which is all a transmit-only PHY can do. Note that
//! **chrony and systemd-timesyncd do not implement broadcast client mode at all** — the reference
//! `ntpd` does, with `broadcastclient` (and, since we cannot answer the calibration exchange it
//! would like to make, `disable auth` and an explicit `broadcastdelay`).

use core::cell::RefCell;

use defmt::{info, warn};
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::peripherals::{PIO0, PIO1, UART0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::uart::{
    BufferedInterruptHandler, BufferedUart, BufferedUartRx, Config as UartConfig,
};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Instant, Timer};
use static_cell::StaticCell;

use ntp_refclock::packet::PACKET_LEN;
use ntp_refclock::server::{ClockState, ServeDecision, ServerConfig, broadcast};
use pico_10base_t::embassy::Tx10BaseT;
use pico_10base_t::frame::{Ipv4Addr, MacAddr, UdpFrameSpec, build_udp_frame, frame_len};
use pico_10base_t::phy::{NLP_INTERVAL_US, encode_frame, encoded_words};
use rp_pps::PpsGpsdo;
use rp_pps::embassy::{TimedPpsCapture, run_capture, run_nmea};

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    // The Ethernet serialiser's symbol feed. This firmware uses exactly one DMA channel.
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>;
});

// --- Network identity -------------------------------------------------------------------------
//
// Fixed, because a transmit-only station cannot DHCP or answer ARP. A client identifies the source
// by its IP, so it has to be stable and it has to be on the client's subnet.

/// Locally-administered MAC (the `02:` prefix marks it as such, so it cannot collide with a real
/// assignment).
const SRC_MAC: MacAddr = MacAddr([0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEE]);
/// Our address. **Change this to something on your LAN.**
const SRC_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 1, 200);
/// Limited broadcast. Never forwarded by a router, and far easier to debug than multicast — no
/// IGMP, no switch snooping, no per-client group membership. `Ipv4Addr::NTP_MULTICAST` (224.0.1.1)
/// is the other legitimate choice once this works.
const DST_IP: Ipv4Addr = Ipv4Addr::BROADCAST;
const DST_MAC: MacAddr = MacAddr::BROADCAST;
const NTP_PORT: u16 = 123;

// --- Server policy ----------------------------------------------------------------------------

/// How long it takes from `send()` returning control to the first bit reaching the wire.
///
/// **Currently a placeholder.** It has to be measured — a broadcast client cannot sound the path,
/// so this offset lands directly in every client's clock. Until then the transmit timestamp is
/// systematically early by whatever this really is, and `PRECISION` is set to say so.
const TX_LEAD_NS: i64 = 0;

const CFG: ServerConfig = ServerConfig {
    // log2 seconds of the resolution at which we can actually *timestamp* a transmission — not the
    // oscillator's. Deliberately pessimistic at ~1 µs until TX_LEAD_NS is measured; over-claiming
    // here corrupts a client's source selection, which is worse than under-claiming.
    precision: -20,
    // Broadcast interval, log2 seconds: one per second.
    poll: 0,
    reference_id: *b"GPS\0",
    // Our own uncertainty when freshly disciplined. Conservative for the same reason as `precision`.
    base_dispersion_ns: 1_000_000,
    // Bound on fractional frequency error during holdover. The crystal measures ~0.6 ppm off and
    // the estimator tracks it to the ppb level, so 1 ppm is a safe envelope.
    holdover_drift_ppb: 1_000,
    // Stop serving after an hour without a PPS edge.
    max_holdover_ns: 3_600 * 1_000_000_000,
};

const FRAME_LEN: usize = frame_len(PACKET_LEN);
const SYMBOL_WORDS: usize = encoded_words(FRAME_LEN);

/// The disciplined clock. The two rp-pps runners write it; the NTP task reads it.
static CLOCK: BlockingMutex<CriticalSectionRawMutex, RefCell<PpsGpsdo>> =
    BlockingMutex::new(RefCell::new(PpsGpsdo::new()));

/// `Instant` as nanoseconds (µs resolution) — the query timebase for the disciplined clock.
fn now_ns() -> u64 {
    Instant::now().as_micros() * 1000
}

#[embassy_executor::task]
async fn pps_task(capture: TimedPpsCapture<'static, PIO0, 0>) {
    run_capture(capture, &CLOCK, now_ns).await
}

#[embassy_executor::task]
async fn nmea_task(rx: BufferedUartRx) {
    run_nmea(rx, &CLOCK).await
}

/// Read everything the server policy needs, in one lock so the values are mutually consistent.
fn clock_state() -> (Option<i64>, ClockState) {
    let q = now_ns();
    CLOCK.lock(|g| {
        let g = g.borrow();
        let now = g.now_from_query_ns(q);
        let holdover_ns = g.holdover_ns(q);
        (
            now,
            ClockState {
                // NTP's reference timestamp is when the clock was last corrected, which is exactly
                // `now` minus how long we have been extrapolating.
                last_update_unix_ns: now.map(|n| n - holdover_ns as i64),
                holdover_ns,
                frequency_locked: g.frequency_locked(),
            },
        )
    })
}

/// Broadcast one NTP packet per UTC second, and keep the link alive in between.
///
/// Both jobs live in one task because they share the transmitter. Interleaving them here also keeps
/// the ordering obvious: a link pulse is never emitted while a frame is going out.
#[embassy_executor::task]
async fn ntp_task(mut tx: Tx10BaseT<'static, PIO1, 0>) {
    let mut frame = [0u8; FRAME_LEN];
    let mut symbols = [0u32; SYMBOL_WORDS];
    let mut ip_id: u16 = 0;
    let mut sent: u32 = 0;
    let nlp_ms = (NLP_INTERVAL_US / 1000) as u64;

    loop {
        let (now, state) = clock_state();

        // Without a UTC epoch there is no second boundary to aim at. Keep the link up and wait.
        let Some(now) = now else {
            tx.link_pulse();
            Timer::after_millis(nlp_ms).await;
            continue;
        };

        // Time to the next whole UTC second, in disciplined nanoseconds.
        let until_next = 1_000_000_000 - now.rem_euclid(1_000_000_000);
        if until_next > (nlp_ms * 1_000_000) as i64 {
            tx.link_pulse();
            Timer::after_millis(nlp_ms).await;
            continue;
        }

        // Aim the *frame's first bit* at the second boundary, so the transmit timestamp we write is
        // the time the packet is actually on the wire rather than when we started thinking about it.
        let target_unix_ns = now + until_next;
        let sleep_ns = (until_next - TX_LEAD_NS).max(0);
        Timer::after_micros((sleep_ns / 1000) as u64).await;

        // Re-read: the policy gate must reflect the state at transmission, not a second ago.
        let (_, state_now) = clock_state();
        let state = if state_now.last_update_unix_ns.is_some() {
            state_now
        } else {
            state
        };

        match broadcast(&CFG, &state, target_unix_ns) {
            ServeDecision::Silent(reason) => {
                // Deliberately quiet: an unsynchronised beacon is only noise to a client that
                // cannot interrogate us. Say why on RTT so the reason is visible.
                warn!("NTP silent: {}", defmt::Debug2Format(&reason));
                tx.link_pulse();
                Timer::after_millis(nlp_ms).await;
            }
            ServeDecision::Serve(packet) => {
                ip_id = ip_id.wrapping_add(1);
                let payload = packet.encode();
                let spec = UdpFrameSpec {
                    src_mac: SRC_MAC,
                    dst_mac: DST_MAC,
                    src_ip: SRC_IP,
                    dst_ip: DST_IP,
                    src_port: NTP_PORT,
                    dst_port: NTP_PORT,
                    ip_id,
                    // Broadcast is link-local; one hop is all it may take.
                    ttl: 1,
                    payload: &payload,
                };
                let Some(len) = build_udp_frame(&spec, &mut frame) else {
                    warn!("NTP frame buffer too small");
                    continue;
                };
                let Some(words) = encode_frame(&frame[..len], &mut symbols) else {
                    warn!("NTP symbol buffer too small");
                    continue;
                };

                let before = now_ns();
                tx.send(&symbols[..words]).await;
                let after = now_ns();

                sent = sent.wrapping_add(1);
                // Everything a host-side measurement needs to line its receive timestamps up with
                // what we believed we were sending, and to see the scheduling error separately from
                // the path delay.
                info!(
                    "NTPTX n={} target_unix_ns={} tx_lead_ns={} dma_us={} bytes={} words={} disp_ns={} holdover_ns={}",
                    sent,
                    target_unix_ns,
                    TX_LEAD_NS,
                    (after - before) / 1000,
                    len,
                    words,
                    ntp_refclock::server::root_dispersion_ns(&CFG, state.holdover_ns),
                    state.holdover_ns,
                );
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("pico-ntp: Stratum-1 NTP broadcast (NMEA UART0/GP1 @9600, PPS GP2, 10BASE-T GP16/GP17)");

    // PIO0: the 1PPS capture that disciplines the clock.
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    let capture = TimedPpsCapture::new(&mut common, sm0, p.PIN_2, clk_sys_freq());

    // PIO1: the Ethernet serialiser. A separate PIO block so the two never contend for a state
    // machine or for instruction memory.
    let Pio {
        common: mut eth_common,
        sm0: eth_sm,
        ..
    } = Pio::new(p.PIO1, Irqs);
    let dma = embassy_rp::dma::Channel::new(p.DMA_CH0, Irqs);
    let tx = Tx10BaseT::new(
        &mut eth_common,
        eth_sm,
        p.PIN_16, // TX−
        p.PIN_17, // TX+
        dma,
        clk_sys_freq(),
    );

    // UART0: the receiver's NMEA. TX is unused — this firmware does not reconfigure the module.
    static TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let mut config = UartConfig::default();
    config.baudrate = 9600;
    let uart = BufferedUart::new(
        p.UART0,
        p.PIN_0,
        p.PIN_1,
        Irqs,
        TX_BUF.init([0; 16]),
        RX_BUF.init([0; 256]),
        config,
    );
    let (_uart_tx, uart_rx) = uart.split();

    spawner.spawn(pps_task(capture).unwrap());
    spawner.spawn(nmea_task(uart_rx).unwrap());
    spawner.spawn(ntp_task(tx).unwrap());

    // Report the disciplined clock once a second, independently of whether we are serving, so a
    // silent server can be told apart from a stopped one.
    loop {
        Timer::after_secs(1).await;
        let q = now_ns();
        let (now, ppb, locked, holdover) = CLOCK.lock(|g| {
            let g = g.borrow();
            (
                g.now_from_query_ns(q),
                g.freq_ppb(),
                g.frequency_locked(),
                g.holdover_ns(q),
            )
        });
        match now {
            Some(now) => info!(
                "TIME unix_ns={} ppb={} locked={} holdover_ms={}",
                now,
                ppb,
                locked as u8,
                holdover / 1_000_000
            ),
            None => info!("TIME waiting for a UTC epoch (no PPS+NMEA pairing yet)"),
        }
    }
}
