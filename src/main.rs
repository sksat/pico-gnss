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
//! - `TIME unix_ns=<n> ppb=<p> holdover_ms=<h> locked=<0|1>`  (GPSDO の規律 UTC)
//!
//! ## PPS タイムスタンプは PIO ハードキャプチャ (~16ns 分解能)
//! ソフト (embassy Input + Instant) はジッタ ~9µs が床 (M0+ の critical-section 全 IRQ マスク)。
//! PIO0 SM0 で PPS エッジを sysclk 2 サイクル(=16ns)分解能でラッチし、間隔を ns で得る。
//!
//! ## GPSDO (GPS 規律発振器)
//! PIO の精密な PPS 間隔から RP2040 水晶の周波数オフセット (ppb) を [`DisciplinedClock`] が
//! EMA 推定し、UTC エポックと合わせて規律 UTC を提供する。**PPS が切れている間 (holdover)
//! も推定周波数で外挿**して時刻を保つ。`time_task` が 1Hz で規律 UTC を `TIME` 行に出す。

use core::cell::RefCell;
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
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, BufferedUartTx, Config as UartConfig};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Instant, Timer};
use embedded_io_async::{Read, Write};
use heapless::String;
use static_cell::StaticCell;

use pico_gnss::{
    civil_to_unix, parse_ddmmyy, parse_hhmmss, DisciplinedClock, NmeaLineAssembler, PpsEvent,
    PpsTracker,
};

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

/// AE-GNSS-EXTANT (GYSFFMANC) のデフォルトボーレート 9600。
const GNSS_BAUD: u32 = 9600;

/// PIO 自走カウンタの 1 tick = 2 サイクル。
const PIO_CYCLES_PER_TICK: u64 = 2;
/// 規律 PPS 生成プログラム 1 周の固定オーバーヘッド (clk_sys サイクル)。X = 周期 - これ。
/// ループバック計測で実測 1e9ns になるよう調整する値 (初期推定)。
const GEN_OVERHEAD: i64 = 10;

/// 起動時にモジュールへ送る PMTK 設定 (チェックサムは送信時に計算)。
/// 実機で各コマンドに `$PMTK001,<cmd>,3`(成功) が返ることを確認済 (チップは MT3333 系)。
/// 注: SBAS は地域依存。日本の MSAS は 2020 終了済なので fix quality は 1 のまま
///     (WAAS/EGNOS 圏では有効)。GYSFFMANC は QZSS SLAS 非対応で sub-meter は不可。詳細は NOTES.md。
const PMTK_INIT: &[&str] = &[
    "PMTK313,1", // SBAS 探索を有効化 (WAAS/EGNOS 圏で有効)
    "PMTK301,2", // DGPS 補正源 = SBAS
    "PMTK286,1", // AIC (アクティブ干渉除去) 有効化
    // NMEA 出力: GLL/RMC/VTG/GGA/GSA/GSV(各1) + GST(測位σ, field7) + ZDA(field17) を有効化。
    // フィールド順: GLL,RMC,VTG,GGA,GSA,GSV,GRS,GST,(res×5),MALM,MEPH,MDGP,MDBG,ZDA,MCHN。
    "PMTK314,1,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,0,1,0",
    "PMTK605", // FW バージョン照会 → $PMTK705 で返る (ACK は無く応答が版情報)
];

/// PPS エッジの (PIO ns 時刻, Instant ns 時刻) を pps_task → main へ渡す。
/// PIO 時刻は ns 精度 (err/エポック計算用)、Instant は連続クエリ (ticker/holdover) のアンカー用。
static PPS_TS: Signal<CriticalSectionRawMutex, (u64, u64)> = Signal::new();

/// GPSDO の規律クロック。pps_task が周波数を、main が UTC エポックを更新し、time_task が読む。
static CLOCK: BlockingMutex<CriticalSectionRawMutex, RefCell<DisciplinedClock>> =
    BlockingMutex::new(RefCell::new(DisciplinedClock::new()));

/// Instant を ns に (µs 分解能)。
fn now_local_ns() -> u64 {
    Instant::now().as_micros() * 1000
}

/// `$<payload>*<csum>\r\n` を組み立てて送る。
async fn send_pmtk<W: Write>(tx: &mut W, payload: &str) {
    let mut cs = 0u8;
    for b in payload.as_bytes() {
        cs ^= *b;
    }
    // PMTK314 (NMEA 出力設定) は ~51 文字になるので余裕を持って 96。溢れると truncate されて
    // 不完全コマンドになり、モジュールに拒否される (実際にハマった)。
    let mut line: String<96> = String::new();
    if write!(line, "${}*{:02X}\r\n", payload, cs).is_ok() {
        let _ = tx.write_all(line.as_bytes()).await;
    } else {
        warn!("send_pmtk: line too long: {=str}", payload);
    }
}

/// PIO がラッチした PPS エッジを読み、間隔を ns で求めて出す + 周波数を規律するタスク。
#[embassy_executor::task]
async fn pps_task(mut sm: StateMachine<'static, PIO0, 0>) {
    let ns_per_tick: u64 = PIO_CYCLES_PER_TICK * 1_000_000_000 / clk_sys_freq() as u64; // 16 @125MHz
    let mut tracker = PpsTracker::new();
    let mut last_x: Option<u32> = None;
    let mut edge_ns: u64 = 0;

    loop {
        let x = sm.rx().wait_pull().await; // エッジ時の自走ダウンカウンタ値
        // エポックのアンカー用に Instant をエッジ直後に読む (µs ジッタは絶対オフセットのみに効く)。
        let inst_ns = now_local_ns();

        let interval_ns = match last_x {
            Some(lx) => lx.wrapping_sub(x) as u64 * ns_per_tick, // ダウンカウンタ: prev - curr
            None => 0,
        };
        last_x = Some(x);
        edge_ns += interval_ns;

        // PIO の ns 精度時刻 (edge_ns) と Instant を main へ。err/エポックは edge_ns 基準で ns 精度。
        PPS_TS.signal((edge_ns, inst_ns));

        // PIO の精密な間隔で水晶の周波数オフセットを規律。
        if interval_ns > 0 {
            CLOCK.lock(|c| c.borrow_mut().update_freq(interval_ns as i64));
        }

        let count = tracker.count() + 1;
        let interval_us = interval_ns / 1000;
        let (state, missed): (&str, u32) = match tracker.record(edge_ns / 1000) {
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

/// モジュール設定 (PMTK) を送るタスク。**RX 受信 (main) をブロックしないよう別タスクにする**
/// — 同じループで送ると Timer/送信中に RX が読まれず、届く ACK が RX バッファ溢れで消える。
#[embassy_executor::task]
async fn config_task(mut tx: BufferedUartTx) {
    // 起動直後はモジュールが PMTK を受け付けないので十分待ってから、間隔も空けて送る。
    // 起動直後 (~1s) は UART が同期せず framing が多発しモジュールも未準備なので十分待つ。
    Timer::after_millis(2000).await;
    for cmd in PMTK_INIT {
        send_pmtk(&mut tx, cmd).await;
        Timer::after_millis(600).await;
    }
    info!("pico-gnss: PMTK config sent ({} cmds)", PMTK_INIT.len());
}

/// 1Hz タスク: GPSDO の規律 UTC を出す + 規律 PPS 生成 (SM1) の周期を ppb 補正で更新する。
/// PPS が切れていても周波数外挿で時刻/周期を保つ (holdover)。
#[embassy_executor::task]
async fn time_task(mut sm_gen: StateMachine<'static, PIO0, 1>) {
    let clk = clk_sys_freq() as i64;
    loop {
        Timer::after_secs(1).await;
        let local = now_local_ns();
        let (now, ppb, holdover, locked) = CLOCK.lock(|c| {
            let c = c.borrow();
            // 連続クエリは Instant 系 (サブ秒は µs 精度)。エポックの絶対対応は Instant アンカー。
            (c.now_from_instant_ns(local), c.freq_ppb(), c.holdover_ns(local), c.is_locked())
        });
        if let Some(now) = now {
            info!(
                "TIME unix_ns={} ppb={} holdover_ms={} locked={}",
                now,
                ppb,
                holdover / 1_000_000,
                locked as u8
            );
        }
        // 規律 PPS 生成 SM の周期を更新: 1 真秒 = clk×(1+ppb/1e9) clk_sys サイクル。
        // プログラム 1 周 = X + GEN_OVERHEAD サイクルなので X = 周期 - overhead。
        let period = clk + clk * ppb / 1_000_000_000 - GEN_OVERHEAD;
        let _ = sm_gen.tx().try_push(period as u32);
    }
}

/// ループバック (GP3→GP4) で規律 PPS 出力の周期を計測するタスク (SM2 capture)。
/// GP4 にジャンパで戻すと、PIO ハード生成した出力エッジを GPS PPS と同じ手法で ns 計測できる。
#[embassy_executor::task]
async fn gen_capture_task(mut sm: StateMachine<'static, PIO0, 2>) {
    let ns_per_tick: u64 = PIO_CYCLES_PER_TICK * 1_000_000_000 / clk_sys_freq() as u64;
    let mut last_x: Option<u32> = None;
    let mut count: u32 = 0;
    loop {
        let x = sm.rx().wait_pull().await;
        if let Some(lx) = last_x {
            let interval_ns = lx.wrapping_sub(x) as u64 * ns_per_tick;
            count += 1;
            info!(
                "PPSGEN count={} interval_ns={} dev_ns={}",
                count,
                interval_ns,
                interval_ns as i64 - 1_000_000_000
            );
        }
        last_x = Some(x);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("pico-gnss: start (NMEA on UART0/GP1 @ {} baud, PPS on GP2 via PIO, GPSDO)", GNSS_BAUD);

    // PPS on GP2 を PIO0 SM0 でハードキャプチャ (自走ダウンカウンタを立ち上がりで push)。
    let Pio { mut common, mut sm0, mut sm1, mut sm2, .. } = Pio::new(p.PIO0, Irqs);
    let prg = pio_asm!(
        ".wrap_target",
        "low:",
        "    jmp pin rising",
        "    jmp x-- low",
        "rising:",
        "    in x, 32",
        "    push noblock",
        "high:",
        "    jmp x-- highchk",
        "highchk:",
        "    jmp pin high",
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

    // SM1: 規律 PPS 生成を GP3 へ。pull noblock で周期を保持し、毎周 X サイクル low + 短い high。
    // CPU (time_task) が ppb 補正した周期を毎秒 push。エッジは PIO ハード生成なので ns クリーン。
    // 周期は X に保持 (pull noblock は FIFO 空のとき X を OSR に再ロードする仕様)。
    // カウントダウンは Y を使う ← X を delay で潰すと 2 発目以降が壊れる (ハマった)。
    let gen_prg = pio_asm!(
        "    set pindirs, 1", // GP3 を出力に (起動時 1 回; SET ピン=GP3)
        ".wrap_target",
        "    pull noblock",  // OSR = 新周期 (FIFO 空なら X = 保持中の周期)
        "    mov x, osr",    // X = 周期 (保持; カウントダウンでは触らない)
        "    mov y, osr",    // Y = カウントダウン用コピー
        "    set pins, 1 [10]", // 立ち上がりエッジ + ~88ns high
        "    set pins, 0",   // 立ち下がり
        "delay:",
        "    jmp y-- delay", // Y+1 サイクル low (X は保持)
        ".wrap",
    );
    let gen_loaded = common.load_program(&gen_prg.program);
    let gen_pin = common.make_pio_pin(p.PIN_3);
    sm1.set_pin_dirs(PioDirection::Out, &[&gen_pin]);
    let mut gen_cfg = PioConfig::default();
    gen_cfg.use_program(&gen_loaded, &[]);
    gen_cfg.set_set_pins(&[&gen_pin]);
    sm1.set_config(&gen_cfg);
    let _ = sm1.tx().try_push((clk_sys_freq() as i64 - GEN_OVERHEAD) as u32); // 初期周期 (ppb=0)
    sm1.set_enable(true);

    // SM2: ループバック (GP3→GP4 ジャンパ) で出力エッジを GPS PPS と同じ手法で捕捉・計測。
    let lb_pin = common.make_pio_pin(p.PIN_4);
    sm2.set_pin_dirs(PioDirection::In, &[&lb_pin]);
    let mut lb_cfg = PioConfig::default();
    lb_cfg.use_program(&loaded, &[]); // capture プログラムを再利用
    lb_cfg.set_jmp_pin(&lb_pin);
    sm2.set_config(&lb_cfg);
    sm2.set_enable(true);

    spawner.spawn(time_task(sm1).unwrap());
    spawner.spawn(gen_capture_task(sm2).unwrap());

    // UART0: TX=GP0 (モジュール設定送信), RX=GP1 (NMEA 受信)。
    static TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let tx_buf = TX_BUF.init([0; 64]);
    let rx_buf = RX_BUF.init([0; 256]);
    let mut config = UartConfig::default();
    config.baudrate = GNSS_BAUD;
    let uart = BufferedUart::new(p.UART0, p.PIN_0, p.PIN_1, Irqs, tx_buf, rx_buf, config);
    let (tx, mut rx) = uart.split();

    // モジュール設定は別タスクで送る (main は RX を読み続ける)。
    spawner.spawn(config_task(tx).unwrap());

    let mut assembler = NmeaLineAssembler::new();
    let mut read_buf = [0u8; 64];
    let mut pending: Option<(u64, u64)> = None; // (PIO ns, Instant ns)

    loop {
        let n = match rx.read(&mut read_buf).await {
            Ok(n) => n,
            // Framing/Overrun は起動直後やモジュール再設定時に多発する。毎回 warn! すると
            // RTT が溢れて他のログ (ACK/NMEA) を trim してしまうので、黙って継続する
            // (assembler は次の '$' で再同期する)。
            Err(_) => continue,
        };

        // PIO がラッチした最新 PPS エッジ (PIO ns, Instant ns) を覚えておく。
        if let Some(e) = PPS_TS.try_take() {
            pending = Some(e);
        }

        for &b in &read_buf[..n] {
            let Some(sentence) = assembler.push(b) else {
                continue;
            };
            let Ok(s) = core::str::from_utf8(sentence) else {
                continue;
            };
            info!("NMEA {=str}", s);

            // FW バージョン応答 ($PMTK705) は専用行で出す (talker が長く NMEA 抽出に乗らないため)。
            if s.starts_with("$PMTK705") {
                info!("FW {=str}", s);
            }

            // RMC (日付+時刻) と直近 PPS エッジを対応付けて UTC エポックを固定する。
            if s.get(3..6) == Some("RMC") {
                let time = s.split(',').nth(1).and_then(parse_hhmmss);
                let date = s.split(',').nth(9).and_then(parse_ddmmyy);
                if let (Some((h, mi, se)), Some((d, mo, y)), Some((pio_ns, inst_ns))) =
                    (time, date, pending)
                {
                    let unix_s = civil_to_unix(y as i64, mo as i64, d as i64, h as i64, mi as i64, se as i64);
                    let target = unix_s * 1_000_000_000;
                    let (ppb, err_ns) = CLOCK.lock(|c| {
                        let mut c = c.borrow_mut();
                        // 補正後の時刻精度: このエッジの UTC を「更新前のクロック (前回エポック+周波数)」で
                        // 予測し、実際の UTC 秒との差を取る = 1 秒 holdover 後の残差。PIO 時刻 (ns 精度) で
                        // 計算するので err も ns 精度 (Instant の µs ジッタに汚されない)。
                        let err = c.now_ns(pio_ns).map(|pred| pred - target).unwrap_or(0);
                        c.update_epoch(pio_ns, inst_ns, target);
                        (c.freq_ppb(), err)
                    });
                    // SYNC (webapp 互換): drift は規律された ppb を µs/s で。err_ns は補正後の予測残差 (ns)。
                    info!(
                        "SYNC pps_local_us={} unix_s={} drift_us={} err_ns={}",
                        pio_ns / 1000,
                        unix_s,
                        ppb / 1000,
                        err_ns
                    );
                }
            }
        }
    }
}
