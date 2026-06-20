//! On-target ベンチ: RMC 1 文からの time+date 抽出コストを in-house vs [`nmea`] crate で比較する。
//!
//! 通常の `cargo build` / `cargo test` では**作られない** (`bench` feature 必須)。
//! 実機 (RP2040 + probe-rs 対応プローブ) で走らせて再現する:
//!
//! ```text
//! cargo run --release --features bench --bin bench_nmea_parse
//! ```
//!
//! 結果は defmt(RTT)に出力。実測例 (RP2040, Cortex-M0+ @125MHz):
//! in-house ≈ 37 µs/iter, nmea ≈ 619 µs/iter (≈ 17x)。どちらも 1Hz では無視できるが、
//! M0+ は FPU 無しのため nmea の lat/lon/speed/course の soft-float 解析で比が開く。
//! nmea ループは ~13s ほどかかる (N×619µs) ので、出力まで少し待つこと。
#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

// 教科書的 RMC (checksum *6A 正当, ダミー座標)。
const RMC: &str = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
const N: u32 = 20_000;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_rp::init(Default::default());
    Timer::after_millis(800).await; // RTT 接続待ち
    info!("bench start: RMC parse x{} (in-house then nmea)", N);

    // in-house: split + parse_hhmmss / parse_ddmmyy (checksum 非検証)
    let t0 = Instant::now();
    let mut acc = 0u64;
    for _ in 0..N {
        let s = core::hint::black_box(RMC);
        let time = s.split(',').nth(1).and_then(pico_gnss::parse_hhmmss);
        let date = s.split(',').nth(9).and_then(pico_gnss::parse_ddmmyy);
        if let (Some((h, mi, se)), Some((d, mo, y))) = (time, date) {
            acc += h as u64 + mi as u64 + se as u64 + d as u64 + mo as u64 + y as u64;
        }
    }
    let ih_us = t0.elapsed().as_micros();
    core::hint::black_box(acc);

    // nmea crate: parse_nmea_sentence (checksum 検証) + parse_rmc (float 解析込み)
    let t1 = Instant::now();
    let mut acc2 = 0u64;
    for _ in 0..N {
        let s = core::hint::black_box(RMC);
        if let Ok(sent) = nmea::parse_nmea_sentence(s) {
            if let Ok(rmc) = nmea::sentences::parse_rmc(sent) {
                if rmc.fix_time.is_some() && rmc.fix_date.is_some() {
                    acc2 += 1;
                }
            }
        }
    }
    let nm_us = t1.elapsed().as_micros();
    core::hint::black_box(acc2);

    info!("in-house: {} us total = {} ns/iter", ih_us, ih_us * 1000 / N as u64);
    info!("nmea    : {} us total = {} ns/iter", nm_us, nm_us * 1000 / N as u64);
    info!("ratio x100 (nmea / in-house): {}", nm_us * 100 / ih_us);
    loop {
        Timer::after_secs(3600).await;
    }
}
