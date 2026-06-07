#![no_std]
#![no_main]

//! GNSS 受信テスト firmware (RP2040 / Raspberry Pi Pico)。
//!
//! - UART0 RX = GP1 に GNSS モジュール (秋月 AE-GNSS-EXTANT+ANT_SET / GYSFFMANC) の NMEA TX。
//! - UART0 TX = GP0 → モジュール RX (任意。起動時に PMTK 設定コマンドを送る)。
//! - PPS = GP2。
//!
//! ## 出力 (defmt-rtt → probe-rs → webapp/server.ts が抽出)
//! - `NMEA $GxXXX,...*hh`
//! - `PPS count=<n> interval_us=<us> interval_ns=<ns> state=<...> missed=<m>`
//! - `SYNC pps_local_us=<t> unix_s=<s> drift_us=<d>`
//!
//! ## PPS タイムスタンプは PIO ハードキャプチャ
//! ソフト (embassy Input + Instant) でエッジを刻むと、Cortex-M0+ の critical-section が
//! 全 IRQ をマスクするためジッタ ~9µs が床になる。そこで **PIO で PPS エッジを sysclk
//! 2 サイクル (=16ns @125MHz) 分解能でラッチ**し、CPU/割込のレイテンシを完全に排除する。
//! PIO の自走ダウンカウンタ X をエッジで FIFO に push、CPU は連続する値の差から間隔を ns で得る。
//! (X は ~68s で 1 周し、0 通過時に低位相ループで稀に誤キャプチャが出るので、host 側で
//!  範囲外の間隔は除外する。)

use core::fmt::Write as _;

use defmt::{info, warn};
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::peripherals::{PIO0, UART0};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Config as PioConfig, Direction as PioDirection, InterruptHandler as PioInterruptHandler, Pio,
    StateMachine,
};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config as UartConfig};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_io_async::{Read, Write};
use heapless::String;
use static_cell::StaticCell;

use pico_gnss::{parse_ddmmyy, parse_hhmmss, NmeaLineAssembler, PpsEvent, PpsTimeSync, PpsTracker};

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

/// AE-GNSS-EXTANT (GYSFFMANC) のデフォルトボーレート 9600。
const GNSS_BAUD: u32 = 9600;

/// PIO 自走カウンタの 1 tick = 2 サイクル。
const PIO_CYCLES_PER_TICK: u64 = 2;

/// 起動時にモジュールへ送る PMTK 設定 (チェックサムは送信時に計算)。
/// 実機で各コマンドに `$PMTK001,<cmd>,3`(成功) が返ることを確認済 (チップは MT3333 系)。
/// 注: SBAS は地域依存。日本の MSAS は 2020 終了済なので fix quality は 1 のまま
///     (WAAS/EGNOS 圏では有効)。GYSFFMANC は QZSS SLAS 非対応で sub-meter は不可。詳細は NOTES.md。
const PMTK_INIT: &[&str] = &[
    "PMTK313,1", // SBAS 探索を有効化 (WAAS/EGNOS 圏で有効)
    "PMTK301,2", // DGPS 補正源 = SBAS
    "PMTK286,1", // AIC (アクティブ干渉除去) 有効化
];

/// PPS エッジの local timestamp (µs) を pps_task → main へ渡す。最新値のみ保持。
static PPS_TS: Signal<CriticalSectionRawMutex, u64> = Signal::new();

/// `$<payload>*<csum>\r\n` を組み立てて送る。
async fn send_pmtk<W: Write>(tx: &mut W, payload: &str) {
    let mut cs = 0u8;
    for b in payload.as_bytes() {
        cs ^= *b;
    }
    let mut line: String<48> = String::new();
    let _ = write!(line, "${}*{:02X}\r\n", payload, cs);
    let _ = tx.write_all(line.as_bytes()).await;
}

/// PIO がラッチした PPS エッジ (カウンタ値) を読み、間隔を ns で求めて出すタスク。
#[embassy_executor::task]
async fn pps_task(mut sm: StateMachine<'static, PIO0, 0>) {
    let ns_per_tick: u64 = PIO_CYCLES_PER_TICK * 1_000_000_000 / clk_sys_freq() as u64; // 16 @125MHz
    let mut tracker = PpsTracker::new();
    let mut last_x: Option<u32> = None;
    let mut edge_ns: u64 = 0; // 累積エッジ時刻 (ns)

    loop {
        let x = sm.rx().wait_pull().await; // エッジ時の自走ダウンカウンタ値
        let interval_ns = match last_x {
            // ダウンカウンタなので prev - curr (wrapping)。
            Some(lx) => lx.wrapping_sub(x) as u64 * ns_per_tick,
            None => 0,
        };
        last_x = Some(x);
        edge_ns += interval_ns;

        let edge_us = edge_ns / 1000;
        PPS_TS.signal(edge_us);

        let count = tracker.count() + 1;
        let interval_us = interval_ns / 1000;
        let (state, missed): (&str, u32) = match tracker.record(edge_us) {
            PpsEvent::First => ("First", 0),
            PpsEvent::Locked { .. } => ("Locked", 0),
            PpsEvent::Irregular { missed, .. } => ("Irregular", missed),
        };
        info!(
            "PPS count={} interval_us={} interval_ns={} state={=str} missed={}",
            count, interval_us, interval_ns, state, missed
        );
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("pico-gnss: start (NMEA on UART0/GP1 @ {} baud, PPS on GP2 via PIO)", GNSS_BAUD);

    // PPS on GP2 を PIO0 SM0 でハードキャプチャする。
    // 自走ダウンカウンタ X を 2 サイクル毎に減算しつつ pin を監視し、立ち上がりで X を push。
    let Pio { mut common, mut sm0, .. } = Pio::new(p.PIO0, Irqs);
    let prg = pio_asm!(
        ".wrap_target",
        "low:",
        "    jmp pin rising",   // pin high -> 立ち上がり
        "    jmp x-- low",      // X 減算してループ (低位相 2 cyc/iter)
        "rising:",
        "    in x, 32",         // ISR = X
        "    push noblock",     // FIFO へ
        "high:",
        "    jmp x-- highchk",  // X 減算
        "highchk:",
        "    jmp pin high",     // まだ high ならループ; low なら wrap -> low (高位相 2 cyc/iter)
        ".wrap",
    );
    let loaded = common.load_program(&prg.program);
    let pps_pin = common.make_pio_pin(p.PIN_2);
    sm0.set_pin_dirs(PioDirection::In, &[&pps_pin]);
    let mut pio_cfg = PioConfig::default();
    pio_cfg.use_program(&loaded, &[]);
    pio_cfg.set_jmp_pin(&pps_pin);
    sm0.set_config(&pio_cfg);
    sm0.set_enable(true);
    spawner.spawn(pps_task(sm0).unwrap());

    // UART0: TX=GP0 (モジュール設定送信), RX=GP1 (NMEA 受信)。
    static TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let tx_buf = TX_BUF.init([0; 64]);
    let rx_buf = RX_BUF.init([0; 256]);
    let mut config = UartConfig::default();
    config.baudrate = GNSS_BAUD;
    let uart = BufferedUart::new(p.UART0, p.PIN_0, p.PIN_1, Irqs, tx_buf, rx_buf, config);
    let (mut tx, mut rx) = uart.split();

    // 起動直後にモジュール設定を送る (TX→モジュール RX 配線時に有効)。
    embassy_time::Timer::after_millis(300).await;
    for cmd in PMTK_INIT {
        send_pmtk(&mut tx, cmd).await;
        embassy_time::Timer::after_millis(120).await;
    }
    info!("pico-gnss: sent {} PMTK config commands (SBAS/AIC)", PMTK_INIT.len());

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

        // PIO がラッチした最新 PPS エッジ時刻を取り込む。
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
            info!("NMEA {=str}", s);

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
