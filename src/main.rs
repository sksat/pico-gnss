#![no_std]
#![no_main]

//! GNSS 受信テスト firmware (RP2040 / Raspberry Pi Pico)。
//!
//! - UART0 RX = GP1 に GNSS モジュール (秋月 AE-GNSS-EXTANT+ANT_SET) の NMEA TX を接続。
//! - PPS = GP2。
//!
//! バイトストリームの 1 センテンスへの切り出しは [`pico_gnss::NmeaLineAssembler`]
//! (host テスト済み)、センテンスのパースは `nmea` クレートに委譲する。
//! ログは defmt-rtt 経由で probe-rs (PicoBridge Lite) の RTT に出る。

use defmt::{info, warn};
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::peripherals::UART0;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUartRx, Config as UartConfig};
use embassy_time::Instant;
use embedded_io_async::Read;
use static_cell::StaticCell;

use nmea::Nmea;

use pico_gnss::{NmeaLineAssembler, PpsEvent, PpsTracker};

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

/// AE-GNSS-EXTANT のボーレート。多くの GNSS モジュールのデフォルトは 9600 (要実機確認)。
const GNSS_BAUD: u32 = 9600;

/// PPS (GP2) の立ち上がりエッジを待ち、[`PpsTracker`] で間隔・欠落を判定して出すタスク。
#[embassy_executor::task]
async fn pps_task(mut pps: Input<'static>) {
    let mut tracker = PpsTracker::new();
    loop {
        pps.wait_for_rising_edge().await;
        let now_us = Instant::now().as_micros();
        match tracker.record(now_us) {
            PpsEvent::First => info!("PPS #{}: first pulse", tracker.count()),
            PpsEvent::Locked { interval_us } => {
                info!("PPS #{}: locked, interval = {} us", tracker.count(), interval_us)
            }
            PpsEvent::Irregular {
                interval_us,
                missed,
            } => warn!(
                "PPS #{}: irregular, interval = {} us, missed = {}",
                tracker.count(),
                interval_us,
                missed
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
    let mut nmea = Nmea::default();
    let mut read_buf = [0u8; 64];

    loop {
        let n = match rx.read(&mut read_buf).await {
            Ok(n) => n,
            Err(e) => {
                warn!("uart read error: {:?}", e);
                continue;
            }
        };

        // 生バイトは bring-up 用に debug レベルで残す (DEFMT_LOG=debug で見える)。
        defmt::debug!("uart rx {} bytes: {=[u8]:a}", n, &read_buf[..n]);

        for &b in &read_buf[..n] {
            let Some(sentence) = assembler.push(b) else {
                continue;
            };
            let Ok(s) = core::str::from_utf8(sentence) else {
                continue;
            };
            // チェックサム検証とフィールド分解は nmea クレートに任せる。
            // センテンス種別 ($ttSSS の tt=talker, SSS=種別)。
            let talker = s.get(1..3).unwrap_or("");
            let kind = s.get(3..6).unwrap_or("");
            let _ = nmea.parse(s);

            // 測位サマリ: GGA のときだけ (毎センテンス出すと煩い)。fix_quality 0 = 未測位。
            if kind == "GGA" {
                let quality = s.split(',').nth(6).unwrap_or("");
                let lat = nmea.latitude.unwrap_or(f64::NAN);
                let lon = nmea.longitude.unwrap_or(f64::NAN);
                let sats = nmea.num_of_fix_satellites.unwrap_or(0);
                info!(
                    "GGA fix_quality={=str} sats_used={} lat={} lon={}",
                    quality, sats, lat, lon
                );
            }
            // [診断] GSV: 視野内衛星数と SNR(C/N0)。$xxGSV,total,msg,inView,(prn,el,az,snr)*...
            // SNR は field 7,11,15,... (4 フィールド毎の 4 番目)。fix には ~30dBHz 以上が要る。
            if kind == "GSV" {
                let mut max_snr = 0u8;
                for (i, f) in s.split(',').enumerate() {
                    if i >= 7 && (i - 7) % 4 == 0 {
                        let f = f.split('*').next().unwrap_or(f); // 末尾 *checksum を除去
                        if let Ok(v) = f.parse::<u8>() {
                            if v > max_snr {
                                max_snr = v;
                            }
                        }
                    }
                }
                if s.split(',').nth(2) == Some("1") {
                    let in_view = s.split(',').nth(3).unwrap_or("?");
                    info!("GSV {=str}: {=str} in view", talker, in_view);
                }
                if max_snr > 0 {
                    info!("  {=str}GSV max C/N0 this msg = {} dBHz", talker, max_snr);
                }
            }
        }
    }
}
