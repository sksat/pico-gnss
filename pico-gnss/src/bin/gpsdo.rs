#![no_std]
#![no_main]

//! Minimal GPSDO example: **PPS capture + NMEA → disciplined UTC**, driving rp-pps's
//! [`PpsGpsdo`](rp_pps::PpsGpsdo) state bundle **by hand** (its methods, called from the app's own
//! tasks). The sibling `gpsdo_runner` example does the same with the `run_capture` / `run_nmea`
//! runner tasks instead; the full firmware [`main`](../pico-gnss) adds loopback phase + disciplined
//! PPS output on top.
//!
//! No loopback phase measurement, no disciplined PPS *output*, no receiver configuration — just feed
//! PPS edges and framed NMEA, then read frequency-disciplined, holdover-extrapolated UTC.
//!
//! Wiring: GNSS module NMEA TX → UART0 RX (GP1) @ 9600 baud; module 1PPS → GP2 (PIO capture).
//!
//! ```text
//! pps_task : PpsGpsdo::on_pps_edge(edge, now)  -> frequency discipline + record the edge
//! main     : PpsGpsdo::feed_nmea(sentence)     -> pair RMC with the edge, establish the UTC epoch
//! time_task: PpsGpsdo::now_from_query_ns(now)  -> disciplined UTC, 1 Hz
//! ```

use core::cell::RefCell;

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::peripherals::{PIO0, UART0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config as UartConfig};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Instant, Timer};
use embedded_io_async::Read;
use static_cell::StaticCell;

use rp_pps::embassy::TimedPpsCapture;
use rp_pps::{NmeaLineAssembler, PpsGpsdo};

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

/// The disciplined GPSDO state, shared between the PPS task (frequency + edge), the UART loop
/// (UTC epoch from NMEA) and the 1 Hz reader. `PpsGpsdo` bundles the discipline + PPS↔NMEA pairing.
static CLOCK: BlockingMutex<CriticalSectionRawMutex, RefCell<PpsGpsdo>> =
    BlockingMutex::new(RefCell::new(PpsGpsdo::new()));

/// `Instant` as nanoseconds (µs resolution) — the query timebase for the disciplined clock.
fn now_ns() -> u64 {
    Instant::now().as_micros() * 1000
}

/// Read each hardware-captured PPS edge and feed it to the GPSDO (frequency discipline + record the
/// edge for the next NMEA pairing).
#[embassy_executor::task]
async fn pps_task(mut capture: TimedPpsCapture<'static, PIO0, 0>) {
    loop {
        let edge = capture.next_edge().await;
        let step = CLOCK.lock(|g| g.borrow_mut().on_pps_edge(edge, now_ns()));
        let freq = step.freq.map_or("none", |f| f.as_str());
        info!("PPS interval_ns={} freq={}", edge.interval_ns, freq);
    }
}

/// Print the disciplined UTC once a second (holdover-extrapolated while PPS/NMEA are missing).
#[embassy_executor::task]
async fn time_task() {
    loop {
        Timer::after_secs(1).await;
        let (now, ppb, locked) = CLOCK.lock(|g| {
            let g = g.borrow();
            (
                g.now_from_query_ns(now_ns()),
                g.freq_ppb(),
                g.frequency_locked(),
            )
        });
        if let Some(now) = now {
            info!("TIME unix_ns={} ppb={} locked={}", now, ppb, locked as u8);
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("gpsdo: NMEA on UART0/GP1 @ 9600, PPS on GP2 (PIO)");

    // PPS capture on PIO0 SM0 (GP2), paired with a timeline → timed edges.
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    let capture = TimedPpsCapture::new(&mut common, sm0, p.PIN_2, clk_sys_freq());

    // UART0 RX=GP1 for the receiver's NMEA (TX=GP0 unused here).
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
    let (_tx, mut rx) = uart.split();

    spawner.spawn(pps_task(capture).unwrap());
    spawner.spawn(time_task().unwrap());

    // Frame the NMEA byte stream into sentences and feed each to the GPSDO. An RMC paired with a
    // fresh PPS edge establishes the UTC epoch; `feed_nmea` returns the sync diagnostics to log.
    let mut assembler = NmeaLineAssembler::new();
    let mut buf = [0u8; 64];
    loop {
        let n = match rx.read(&mut buf).await {
            Ok(n) => n,
            Err(_) => continue, // framing/overrun is common at start-up; resync on the next '$'
        };
        for &b in &buf[..n] {
            let Some(sentence) = assembler.push(b) else {
                continue;
            };
            let Ok(s) = core::str::from_utf8(sentence) else {
                continue;
            };
            if let Some(r) = CLOCK.lock(|g| g.borrow_mut().feed_nmea(s)) {
                info!(
                    "SYNC unix_s={} err_ns={} holdover_ms={}",
                    r.unix_ns / 1_000_000_000,
                    r.err_ns,
                    (r.holdover_ns / 1_000_000) as u32
                );
            }
        }
    }
}
