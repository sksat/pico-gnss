#![no_std]
#![no_main]

//! GNSS 受信テスト firmware (RP2040 / Raspberry Pi Pico)。
//!
//! - UART0 RX = GP1 に GNSS モジュール (秋月 AE-GNSS-EXTANT+ANT_SET / GYSFFMANC) の NMEA TX を接続。
//! - PPS = GP2。
//!
//! ## 出力 (defmt-rtt → probe-rs → webapp/server.ts が抽出)
//! - `NMEA $GxXXX,...*hh` : 受信した生 NMEA センテンス (パース・可視化は Web 側)。
//! - `PPS count=<n> interval_us=<us> state=<First|Locked|Irregular> missed=<m>` : PPS エッジ。
//! - `SYNC pps_local_us=<t> unix_s=<s> drift_us=<d>` : PPS 規律された UTC エポック。
//!
//! ## 時刻同期は firmware 側で行う (精度のため)
//! PPS の立ち上がりは UTC 秒境界。その瞬間の local timer 値 (1µs) を、後続 NMEA の
//! UTC 秒と対応付ける ([`PpsTimeSync`])。host 側で同期すると probe/USB のジッタ (数十 ms)
//! が乗るので、エッジを µs で刻める MCU 上で対応付けるのが必須。
//!
//! PPS エッジは専用タスク [`pps_task`] で即座に [`Instant::now`] を取り (レイテンシ最小)、
//! [`Signal`] で main タスクへ渡す。main は NMEA の時刻と突き合わせて `SYNC` を出す。

use defmt::{info, warn};
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::peripherals::UART0;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUartRx, Config as UartConfig};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;
use embedded_io_async::Read;
use static_cell::StaticCell;

use pico_gnss::{parse_ddmmyy, parse_hhmmss, NmeaLineAssembler, PpsEvent, PpsTimeSync, PpsTracker};

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

/// AE-GNSS-EXTANT (GYSFFMANC) のデフォルトボーレート 9600。
const GNSS_BAUD: u32 = 9600;

/// PPS エッジの local timestamp (µs) を pps_task → main へ渡す。最新値のみ保持。
static PPS_TS: Signal<CriticalSectionRawMutex, u64> = Signal::new();

/// PPS (GP2) の立ち上がりエッジを待ち、即座に timestamp を取って main へ送るタスク。
/// 併せて [`PpsTracker`] で間隔/欠落を判定し、PPS 行を出す。
#[embassy_executor::task]
async fn pps_task(mut pps: Input<'static>) {
    let mut tracker = PpsTracker::new();
    loop {
        pps.wait_for_rising_edge().await;
        // エッジ直後に刻む (時刻同期の精度はここのレイテンシで決まる)。
        let now_us = Instant::now().as_micros();
        PPS_TS.signal(now_us);

        let count = tracker.count() + 1;
        match tracker.record(now_us) {
            PpsEvent::First => {
                info!("PPS count={} interval_us={} state=First missed=0", count, 0u64)
            }
            PpsEvent::Locked { interval_us } => {
                info!("PPS count={} interval_us={} state=Locked missed=0", count, interval_us)
            }
            PpsEvent::Irregular {
                interval_us,
                missed,
            } => info!(
                "PPS count={} interval_us={} state=Irregular missed={}",
                count, interval_us, missed
            ),
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("pico-gnss: start (NMEA on UART0/GP1 @ {} baud, PPS on GP2)", GNSS_BAUD);

    // PPS on GP2
    let pps = Input::new(p.PIN_2, Pull::None);
    // embassy-executor 0.10 ではタスク関数が Result<SpawnToken, _> を返す。
    spawner.spawn(pps_task(pps).unwrap());

    // UART0 RX = GP1 (GNSS モジュール TX → Pico RX) で NMEA を受信する。
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let rx_buf = RX_BUF.init([0; 256]);
    let mut config = UartConfig::default();
    config.baudrate = GNSS_BAUD;
    let mut rx = BufferedUartRx::new(p.UART0, Irqs, p.PIN_1, rx_buf, config);

    let mut assembler = NmeaLineAssembler::new();
    let mut timesync = PpsTimeSync::new();
    let mut read_buf = [0u8; 64];

    loop {
        let n = match rx.read(&mut read_buf).await {
            Ok(n) => n,
            Err(e) => {
                warn!("uart read error: {:?}", e);
                continue;
            }
        };

        // pps_task が刻んだ最新 PPS エッジを取り込む。
        if let Some(t) = PPS_TS.try_take() {
            timesync.on_pps(t);
        }

        for &b in &read_buf[..n] {
            let Some(sentence) = assembler.push(b) else {
                continue;
            };
            let Ok(s) = core::str::from_utf8(sentence) else {
                continue;
            };
            // 生 NMEA をそのまま流す。パース・可視化は host (Web) 側。
            info!("NMEA {=str}", s);

            // RMC は日付+時刻を両方持つ。直近 PPS エッジと突き合わせて SYNC を確立する。
            if s.get(3..6) == Some("RMC") {
                let time = s.split(',').nth(1).and_then(parse_hhmmss);
                let date = s.split(',').nth(9).and_then(parse_ddmmyy);
                if let Some((d, mo, y)) = date {
                    timesync.set_date(y, mo, d);
                }
                if let Some((h, mi, se)) = time {
                    if let Some(sp) = timesync.on_time(h, mi, se) {
                        info!(
                            "SYNC pps_local_us={} unix_s={} drift_us={}",
                            sp.pps_local_us, sp.unix_s, sp.drift_us
                        );
                    }
                }
            }
        }
    }
}
