#![no_std]
#![no_main]

//! Minimal GPSDO example using rp-pps's **runner tasks**: [`run_capture`](rp_pps::embassy::run_capture)
//! pumps PPS edges and [`run_nmea`](rp_pps::embassy::run_nmea) pumps the receiver's NMEA, both into a
//! shared [`PpsGpsdo`](rp_pps::PpsGpsdo) — so the application just spawns the two runners and reads
//! disciplined UTC. This is the same minimal GPSDO as the `gpsdo` example, but there the app drives
//! `PpsGpsdo`'s methods by hand; here rp-pps drives them.
//!
//! Wiring: GNSS module NMEA TX → UART0 RX (GP1) @ 9600 baud; module 1PPS → GP2 (PIO capture).
//!
//! ```text
//! run_capture task : PPS edge -> PpsGpsdo::on_pps_edge   (frequency discipline + record the edge)
//! run_nmea    task : NMEA     -> PpsGpsdo::feed_nmea      (pair RMC with the edge -> UTC epoch)
//! main             : PpsGpsdo::now_from_query_ns         (disciplined UTC, 1 Hz)
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
use embassy_rp::uart::{
    BufferedInterruptHandler, BufferedUart, BufferedUartRx, Config as UartConfig,
};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Instant, Timer};
use static_cell::StaticCell;

use rp_pps::PpsGpsdo;
use rp_pps::embassy::{TimedPpsCapture, run_capture, run_nmea};

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

/// The disciplined GPSDO state. The two runner tasks write it (PPS + NMEA); `main` reads it.
static CLOCK: BlockingMutex<CriticalSectionRawMutex, RefCell<PpsGpsdo>> =
    BlockingMutex::new(RefCell::new(PpsGpsdo::new()));

/// `Instant` as nanoseconds (µs resolution) — the query timebase for the disciplined clock.
fn now_ns() -> u64 {
    Instant::now().as_micros() * 1000
}

/// Pump PPS edges into the shared clock (rp-pps runner).
#[embassy_executor::task]
async fn pps_task(capture: TimedPpsCapture<'static, PIO0, 0>) {
    run_capture(capture, &CLOCK, now_ns).await
}

/// Pump the receiver's NMEA into the shared clock (rp-pps runner).
#[embassy_executor::task]
async fn nmea_task(rx: BufferedUartRx) {
    run_nmea(rx, &CLOCK).await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("gpsdo_runner: NMEA on UART0/GP1 @ 9600, PPS on GP2 (PIO)");

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    let capture = TimedPpsCapture::new(&mut common, sm0, p.PIN_2, clk_sys_freq());

    static TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let mut config = UartConfig::default();
    config.baudrate = pico_gnss::mt3333::GNSS_BAUD;
    let mut uart = BufferedUart::new(
        p.UART0,
        p.PIN_0,
        p.PIN_1,
        Irqs,
        TX_BUF.init([0; 16]),
        RX_BUF.init([0; 256]),
        config,
    );
    // 受信機のボーレート設定は firmware の再フラッシュを跨いで残るので、既定決め打ちで開くと、
    // 一度でも引き上げた受信機からは何も受け取れない。設定は変えず、今のレートに合わせるだけ。
    pico_gnss::mt3333::follow_baud(&mut uart).await;
    let (_tx, rx) = uart.split();

    // Spawn the two rp-pps runners; they discipline `CLOCK` on their own.
    spawner.spawn(pps_task(capture).unwrap());
    spawner.spawn(nmea_task(rx).unwrap());

    // The app's only job: read disciplined UTC once a second.
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
