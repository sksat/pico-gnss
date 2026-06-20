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

use defmt::{info, warn};
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::bind_interrupts;
use embassy_rp::interrupt;
use embassy_rp::interrupt::{InterruptExt, Priority};
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::peripherals::{PIO0, UART0};
use embassy_rp::pio::{
    Config as PioConfig, Direction as PioDirection, InterruptHandler as PioInterruptHandler, Pio,
    StateMachine,
};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config as UartConfig};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use embedded_io_async::Read;
use portable_atomic::{AtomicU32, Ordering};
use static_cell::StaticCell;

use gnssdo::{
    civil_to_unix, parse_rmc_time_date, snap_to_second_ns, DisciplinedClock, FreqUpdate, LoopMode,
    NmeaLineAssembler, PhaseLockLoop, PpsEvent, PpsTracker,
};
use rp_pps::embassy::{PpsCapture, PpsOutput};

/// 受信機固有レイヤ (MT3333/GYSFFMANC の PMTK 設定・ボーレート)。別受信機ならここだけ差し替え。
mod mt3333;

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

/// PPS エッジの (PIO ns 時刻, Instant ns 時刻) を pps_task → main へ渡す。
/// PIO 時刻は ns 精度 (err/エポック計算用)、Instant は連続クエリ (ticker/holdover) のアンカー用。
static PPS_TS: Signal<CriticalSectionRawMutex, (u64, u64)> = Signal::new();

/// GPSDO の規律クロック。pps_task が周波数を、main が UTC エポックを更新し、time_task が読む。
static CLOCK: BlockingMutex<CriticalSectionRawMutex, RefCell<DisciplinedClock>> =
    BlockingMutex::new(RefCell::new(DisciplinedClock::new()));

/// 位相制御の測定源。true=PIO ハード位相 (stage②, 本番)。false=旧 Instant 測定 (比較計測用)。
const PHASE_USE_HW: bool = true;
// 位相ループの gains は gnssdo の PhaseLockLoopConfig::DEFAULT に移動した (σ≈35ns 実測チューニング)。
/// 制御項の実験モード: true で **P→PI→PID を ~120 エッジ毎に巡回**し各項の効果を観察 (cfg を PPSGEN に出力)。
/// false で本番 = 常時 PID。
const PHASE_EXPERIMENT: bool = false;
/// 単一制御構成の固定選択 (PHASE_EXPERIMENT=false 時)。0=P,1=PD,2=PI,3=PID,4=PID+Smith(本番)。
/// 各構成をコールドスタートから個別キャプチャして compare 風に重ねる用。
const CTRL_SEL: u32 = 4;

/// 最新の GPS PPS エッジの生カウンタ値 (SM0)。stage② の PIO 位相計測で gen_capture が参照する。
static C0_GPS: AtomicU32 = AtomicU32::new(0);
/// GPS PPS エッジの世代カウンタ。出力エッジ間に進んでいなければ GPS 欠落 = C0_GPS は古い → 補正に使わない。
static C0_GEN: AtomicU32 = AtomicU32::new(0);

/// 実験 harness の cfg (0-4) を PhaseLockLoop の LoopMode へ。本番は 4=PidSmith。
fn cfg_to_mode(cfg: u32) -> LoopMode {
    match cfg {
        0 => LoopMode::P,
        1 => LoopMode::Pd,
        2 => LoopMode::Pi,
        3 => LoopMode::Pid,
        _ => LoopMode::PidSmith,
    }
}

/// gen_capture_task 用の高優先度割込エグゼキュータ。ループバックエッジ→Instant 読みの
/// ウェイクアップ遅延 (thread executor だと UART 処理等で ~ms) を ~µs に下げ、位相測定を精密化する。
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SWI_IRQ_0() {
    // edition 2024: unsafe fn の本体でも unsafe op は明示ブロックが要る。
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

/// Instant を ns に (µs 分解能)。
fn now_local_ns() -> u64 {
    Instant::now().as_micros() * 1000
}

/// PIO がラッチした PPS エッジを読み、間隔を ns で求めて出す + 周波数を規律するタスク。
#[embassy_executor::task]
async fn pps_task(mut capture: PpsCapture<'static, PIO0, 0>) {
    let clk = clk_sys_freq();
    let mut tracker = PpsTracker::new();
    let mut last_x: Option<u32> = None;
    let mut edge_ns: u64 = 0;
    let mut was_locked = false; // 直前エッジが Locked だったか (復帰検出用)

    loop {
        let x = capture.wait_edge().await; // エッジ時の自走ダウンカウンタ値
        C0_GPS.store(x, Ordering::Relaxed); // stage②: 最新 GPS エッジの生カウンタを共有
        C0_GEN.fetch_add(1, Ordering::Relaxed); // 世代++ (gen_capture が欠落検出に使う)
        // エポックのアンカー用に Instant をエッジ直後に読む (µs ジッタは絶対オフセットのみに効く)。
        let inst_ns = now_local_ns();

        let interval_ns = match last_x {
            Some(lx) => rp_pps::interval_ns(lx, x, clk), // ダウンカウンタ: prev - curr (wrap 込み)
            None => 0,
        };
        last_x = Some(x);
        edge_ns += interval_ns;

        // PIO の ns 精度時刻 (edge_ns) と Instant を main へ。err/エポックは edge_ns 基準で ns 精度。
        PPS_TS.signal((edge_ns, inst_ns));

        // 先に PpsTracker で品質を判定する。周波数 EMA は **Locked のエッジでのみ**規律する
        // (Irregular/First の間隔を入れると holdover/PPSGEN の土台が腐る — smart-friend GPT-5.5)。
        let count = tracker.count() + 1;
        let interval_us = interval_ns / 1000;
        let (state, missed): (&str, u32) = match tracker.record(edge_ns / 1000) {
            PpsEvent::First => ("First", 0),
            PpsEvent::Locked { .. } => ("Locked", 0),
            PpsEvent::Irregular { missed, .. } => ("Irregular", missed),
            PpsEvent::NonMonotonic { .. } => ("NonMono", 0),
        };
        let locked = state == "Locked";
        // holdover/Irregular からの復帰 (前回非 Locked → 今回 Locked) を検出したら周波数を検疫する。
        let recovered = locked && !was_locked;
        was_locked = locked;
        // PIO の精密な間隔で水晶の周波数オフセットを規律 (Locked のときだけ、多段ゲート経由)。
        let fu = if interval_ns > 0 && locked {
            CLOCK.lock(|c| {
                let mut c = c.borrow_mut();
                if recovered {
                    c.start_quarantine();
                }
                c.update_freq(interval_ns as i64)
            })
        } else {
            FreqUpdate::GatedQuality
        };
        info!(
            "PPS count={} interval_us={} interval_ns={} state={=str} missed={} freq={}",
            count, interval_us, interval_ns, state, missed, fu.as_str()
        );
    }
}

/// 1Hz タスク: GPSDO の規律 UTC を出す + 規律 PPS 生成 (SM1) の周期を ppb 補正で更新する。
/// PPS が切れていても周波数外挿で時刻/周期を保つ (holdover)。
#[embassy_executor::task]
async fn time_task() {
    loop {
        Timer::after_secs(1).await;
        let local = now_local_ns();
        let (now, ppb, holdover, locked) = CLOCK.lock(|c| {
            let c = c.borrow();
            // 連続クエリは Instant 系 (サブ秒は µs 精度)。エポックの絶対対応は Instant アンカー。
            (c.now_from_query_ns(local), c.freq_ppb(), c.holdover_ns(local), c.is_locked())
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
    }
}

/// 規律 PPS 出力の周期管理 + ループバック計測 (GP3 生成 / GP4 捕捉)。**エッジに同期して**
/// 1 出力エッジにつき 1 回だけ次周期を更新する (= freq 規律 + 位相同期を 1 箇所で・1 サンプル遅れで)。
/// time_task の 1Hz タイマと非同期に補正すると位相が発振したので、ここに集約した。
/// ※ freq/位相規律はループバック (GP3→GP4 ジャンパ) 接続が前提。未接続だと初期周期で自走する。
#[embassy_executor::task]
async fn gen_capture_task(
    mut output: PpsOutput<'static, PIO0, 1>,
    mut sm: StateMachine<'static, PIO0, 2>,
    k: u32, // stage② 較正済みカウンタオフセット (C0−C2)
) {
    let clk = clk_sys_freq();
    let mut last_x: Option<u32> = None;
    let mut count: u32 = 0;
    let mut last_gen: u32 = 0;
    // 出力位相 PLL (制御=gnssdo) と sub-cycle period 生成 (I/O=rp-pps)。gains は PhaseLockLoopConfig。
    let mut pll = PhaseLockLoop::new();
    let mut dither = rp_pps::OutputPeriodDither::new();
    loop {
        let x = sm.rx().wait_pull().await; // x = 出力エッジの生カウンタ (C2_out)
        // GPS PPS の世代。前回の出力エッジから進んでいなければ GPS 欠落 → C0_GPS が古い → 補正に使わない
        // (弱信号で PPS が時々落ちると、古い基準で巨大補正が走り出力が ~100ms 飛ぶのを防ぐ。断中はホールド)。
        let cur_gen = C0_GEN.load(Ordering::Relaxed);
        let fresh = cur_gen != last_gen;
        last_gen = cur_gen;
        // 出力周期。PIO ~68s 周回グリッチの偽エッジは間隔が異常 → 位相補正に使わない。
        let interval_ns = last_x.map(|lx| rp_pps::interval_ns(lx, x, clk) as i64);
        let sane = interval_ns.is_some_and(|iv| (iv - 1_000_000_000).abs() < 300_000_000);
        // stage② PIO ハード位相: 出力エッジ と 最新 GPS エッジ の生カウンタ差 (mod 1秒) を ns に。
        // Instant を介さないので executor のウェイクアップ遅延 (~ms) に汚されず 16ns 分解能。
        let c0 = C0_GPS.load(Ordering::Relaxed);
        let hwphase_ns = rp_pps::loopback_phase_ns(c0, x, k, clk);
        // 旧手法 (比較用 emit のみ): 出力エッジの UTC 時刻 (Instant 経由) → 秒境界からのズレ。
        let (ppb, phase) = CLOCK.lock(|c| {
            let c = c.borrow();
            (c.freq_ppb(), c.now_from_query_ns(now_local_ns()).map(snap_to_second_ns))
        });
        // 制御に使う位相: PIO ハード(stage②)。PHASE_USE_HW=false で旧 Instant 測定に切替 (比較計測用)。
        let ctrl = if PHASE_USE_HW { hwphase_ns } else { phase.unwrap_or(0) };
        // 実験モード: 制御構成を ~130 エッジ毎に巡回し実機で比較 (0=P,1=PD,2=PI,3=PID,4=PID+Smith)。
        // 先頭を PID+Smith にしてコールドスタートをロックしてから P/PD/PI/PID を回す。本番は常に 4。
        let cfg: u32 = if PHASE_EXPERIMENT {
            let ph = (count / 130) % 5;
            if ph == 0 { 4 } else { ph - 1 }
        } else {
            CTRL_SEL // 本番=4(PID+Smith)。0-3 で単一構成に固定キャプチャ (compare 用の公平比較)
        };
        pll.set_mode(cfg_to_mode(cfg));
        // valid = この位相サンプルが信用できるか (GPS 新鮮 + 間隔健全)。!valid は holdover ホールド。
        let valid = fresh && sane && c0 != 0;
        let u = pll.update(ctrl, valid);
        // 周波数 = 水晶推定(ppb) + PLL の I トリム(milli-ppb)。位相補正は dither が cycle に変換して引く。
        let freq_mppb = ppb * 1000 + u.freq_trim_mppb;
        let period = dither.next_period(clk, freq_mppb, u.phase_corr_ns);
        let _ = output.set_period(period);
        if let Some(iv) = interval_ns {
            count += 1;
            // 制御信号を全部出力 → ホストでプラント同定 + ゲイン掃引シミュ + 項別比較ができる。
            // (互換のため既存 5 項 count/interval/dev/phase/hwphase を先頭に残し、cfg/trim/p/d を追記)
            info!(
                "PPSGEN count={} interval_ns={} dev_ns={} phase_ns={} hwphase_ns={} trim_ppb={} cfg={} p_ns={} d_ns={}",
                count, iv, iv - 1_000_000_000, phase.unwrap_or(0), hwphase_ns, u.freq_trim_mppb / 1000, cfg, u.p_corr_ns, u.d_corr_ns
            );
        }
        last_x = Some(x);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("pico-gnss: start (NMEA on UART0/GP1 @ {} baud, PPS on GP2 via PIO, GPSDO)", mt3333::GNSS_BAUD);

    // PPS on GP2 を PIO0 SM0 で rp-pps の PpsCapture でハードキャプチャ (~16ns 分解能)。
    let Pio { mut common, sm0, sm1, mut sm2, .. } = Pio::new(p.PIO0, Irqs);
    let mut capture = PpsCapture::new(&mut common, sm0, p.PIN_2);

    // stage②: SM2 (ループバック位相計測。rp-pps 外の実験機能) は capture と GP2 を共有して生カウンタ差
    // K=C0−C2 を較正する。embassy の set_jmp_pin は &Pin を要求し GP2 は一度しか make できないので、
    // capture.jmp_pin() で GP2 Pin を借り、capture プログラムをもう一枚 load する。
    let lb_loaded = common.load_program(&rp_pps::pps_capture_program());
    let lb_pin = common.make_pio_pin(p.PIN_4);
    sm2.set_pin_dirs(PioDirection::In, &[&lb_pin]);
    let mut cfg_gp2 = PioConfig::default();
    cfg_gp2.use_program(&lb_loaded, &[]);
    cfg_gp2.set_jmp_pin(capture.jmp_pin()); // SM0 と同じ GP2 を捕捉
    sm2.set_config(&cfg_gp2);
    sm2.set_enable(true);
    // SM0 と SM2 で同じ GP2 エッジを捕り、生カウンタ差 K=mean(C0−C2) を rp-pps で較正。
    // 先頭 2 エッジは捨てる。PPS は fix 後のみ出るので各 30s 待ち、出なければ打ち切り。
    let mut k_samples: heapless::Vec<(u32, u32), 8> = heapless::Vec::new();
    for i in 0..7u32 {
        match with_timeout(Duration::from_secs(30), async {
            let c0 = capture.wait_edge().await;
            let c2 = sm2.rx().wait_pull().await;
            (c0, c2)
        })
        .await
        {
            Ok(pair) if i >= 2 => {
                let _ = k_samples.push(pair);
            }
            Ok(_) => {}
            Err(_) => break,
        }
    }
    let k = match rp_pps::calibrate_loopback_offset(k_samples.iter().copied()) {
        Some(k) => {
            info!("PHASE_K calibrated k={} (n={})", k as i32, k_samples.len());
            k
        }
        None => {
            warn!("PHASE_K calibration failed (no PPS)");
            0
        }
    };

    // SM2 を GP4 (ループバック) に切替。set_config は X を触らないので K は有効のまま。
    let mut cfg_gp4 = PioConfig::default();
    cfg_gp4.use_program(&lb_loaded, &[]);
    cfg_gp4.set_jmp_pin(&lb_pin);
    sm2.set_config(&cfg_gp4);

    // SM1: 規律 PPS 生成を GP3 へ (rp-pps の PpsOutput)。初期周期 = ppb=0 の 1Hz。周期は
    // gen_capture_task が毎エッジ set_period で操舵する (servo は firmware 側に残す)。
    let output =
        PpsOutput::new(&mut common, sm1, p.PIN_3, rp_pps::output_period_cycles(clk_sys_freq()));

    // pps_task と gen_capture を高優先度割込エグゼキュータで起動 (ウェイクアップ遅延 ms→µs)。
    interrupt::SWI_IRQ_0.set_priority(Priority::P0);
    let spawner_high = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_0);
    spawner_high.spawn(pps_task(capture).unwrap());
    spawner.spawn(time_task().unwrap());
    spawner_high.spawn(gen_capture_task(output, sm2, k).unwrap());

    // UART0: TX=GP0 (モジュール設定送信), RX=GP1 (NMEA 受信)。
    static TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let tx_buf = TX_BUF.init([0; 64]);
    let rx_buf = RX_BUF.init([0; 256]);
    let mut config = UartConfig::default();
    config.baudrate = mt3333::GNSS_BAUD;
    let uart = BufferedUart::new(p.UART0, p.PIN_0, p.PIN_1, Irqs, tx_buf, rx_buf, config);
    let (tx, mut rx) = uart.split();

    // モジュール設定は別タスクで送る (main は RX を読み続ける)。
    spawner.spawn(mt3333::config_task(tx).unwrap());

    let mut assembler = NmeaLineAssembler::new();
    let mut read_buf = [0u8; 64];
    let mut pending: Option<(u64, u64)> = None; // (PIO ns, Instant ns)
    let mut pending_fresh = false; // 未消費の新しい PPS エッジがあるか

    loop {
        let n = match rx.read(&mut read_buf).await {
            Ok(n) => n,
            // Framing/Overrun は起動直後やモジュール再設定時に多発する。毎回 warn! すると
            // RTT が溢れて他のログ (ACK/NMEA) を trim してしまうので、黙って継続する
            // (assembler は次の '$' で再同期する)。
            Err(_) => continue,
        };

        // PIO がラッチした最新 PPS エッジ (PIO ns, Instant ns) を覚えておく。新しく来たら fresh。
        if let Some(e) = PPS_TS.try_take() {
            pending = Some(e);
            pending_fresh = true;
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
            if mt3333::is_fw_version_response(s) {
                info!("FW {=str}", s);
            }

            // RMC (日付+時刻) と直近 PPS エッジを対応付けて UTC エポックを固定する。
            if let Some(((h, mi, se), (d, mo, y))) = parse_rmc_time_date(s) {
                // 新鮮な (未消費の) PPS エッジがある時だけペアする。PPS 欠落中は stale エッジを
                // 同じ秒に何度もペアして err が ±整数秒の偽値になるため、fresh==true を条件にする。
                if let (Some((pio_ns, inst_ns)), true) = (pending, pending_fresh) {
                    pending_fresh = false; // このエッジは消費した
                    let unix_s = civil_to_unix(y as i64, mo as i64, d as i64, h as i64, mi as i64, se as i64);
                    let target = unix_s * 1_000_000_000;
                    let (ppb, err_ns, fire_ns, hold_ms) = CLOCK.lock(|c| {
                        let mut c = c.borrow_mut();
                        // 補正後の時刻精度 (予測残差) と fire_at_utc の逆予測残差 = GPSDO 自己診断。
                        // どちらも更新前クロックで計算し、複数秒 holdover の整数秒ズレは snap で除く (gnssdo)。
                        let err = c.prediction_residual_ns(pio_ns, target).unwrap_or(0);
                        let fire = c.fire_residual_ns(pio_ns, target).unwrap_or(0);
                        // この sync が前回 sync から何 ms 経っているか (= この err が何秒 holdover の誤差か)。
                        let hold = (c.holdover_ns(inst_ns) / 1_000_000) as u32;
                        c.update_epoch(pio_ns, inst_ns, target);
                        (c.freq_ppb(), err, fire, hold)
                    });
                    // SYNC: err_ns=補正後の予測残差 (timestamp 側)、fire_ns=逆予測残差 (fire_at_utc 側)、holdover_ms=err の holdover 経過。
                    info!(
                        "SYNC pps_local_us={} unix_s={} drift_us={} err_ns={} fire_ns={} holdover_ms={}",
                        pio_ns / 1000,
                        unix_s,
                        ppb / 1000,
                        err_ns,
                        fire_ns,
                        hold_ms
                    );
                }
            }
        }
    }
}
