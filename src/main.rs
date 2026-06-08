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

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::bind_interrupts;
use embassy_rp::interrupt;
use embassy_rp::interrupt::{InterruptExt, Priority};
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
use embassy_time::{with_timeout, Duration, Instant, Timer};
use embedded_io_async::{Read, Write};
use heapless::String;
use portable_atomic::{AtomicU32, Ordering};
use static_cell::StaticCell;

use pico_gnss::{
    civil_to_unix, parse_ddmmyy, parse_hhmmss, snap_to_second_ns, DisciplinedClock,
    NmeaLineAssembler, PpsEvent, PpsTracker,
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

/// 位相制御の測定源。true=PIO ハード位相 (stage②, 本番)。false=旧 Instant 測定 (比較計測用)。
const PHASE_USE_HW: bool = true;
/// 位相同期のデッドバンド (ns)。**0=P を常時効かせ全域で減衰**。Smith 予測子で遅延補償した上で
/// Kp=1/8 (ζ≈0.71) にしたので、deadband 内で P が切れて無減衰になる事態を避ける。
const PHASE_DEADBAND_NS: i64 = 0;
/// ロック判定: 位相がこの内 (1µs) のエッジを LOCK_HOLD 回連続でロック成立。Smith でロックが ±80ns と
/// 締まったので 1µs に下げた (旧 5µs)。
const LOCK_NS: i64 = 1_000;
const LOCK_HOLD: u32 = 5;
/// 外れ値除去 (smart-friend 助言): ロック中にこの幅を超える位相は弱信号の不正 PPS/取りこぼし = 単発 garbage
/// とみなし最大 OUTLIER_MAX 回まで棄却。ロックが ±80ns になったので **3µs に下げ**、弱信号スパイク (4-10µs,
/// 旧 50µs 閾値の下) が trim を蹴るのを防ぐ。それ以上続けば本物の擾乱として再ロック。
const OUTLIER_NS: i64 = 3_000;
const OUTLIER_MAX: u32 = 12;
/// 位相 I 項 (type-II PLL): ロック中、予測位相を周波数トリムに積分してドループ(定常オフセット)を 0 に。
/// 固有周期 ≈ 2π√(PHASE_I_DEN) エッジ。減衰 ζ ≈ Kp/(2√Ki) = (1/8)/(2/√128) ≈ 0.71 (Smith 予測子で遅延
/// 補償済なので公式が成立)。Kp=1/8 + Smith + deadband=0 + 外れ値3µs で **実測 σ~35ns, mean~0** (NOTES)。
const PHASE_I_DEN: i64 = 128;
const PPB_TRIM_MAX: i64 = 3_000;
/// D 項 (微分) ゲイン分母。d_corr = (ctrl − last_ctrl)/PHASE_D_DEN [ns]。位相速度に比例し振動を減衰。
const PHASE_D_DEN: i64 = 4;
/// 制御項の実験モード: true で **P→PI→PID を ~120 エッジ毎に巡回**し各項の効果を観察 (cfg を PPSGEN に出力)。
/// false で本番 = 常時 PID。
const PHASE_EXPERIMENT: bool = false;

/// 最新の GPS PPS エッジの生カウンタ値 (SM0)。stage② の PIO 位相計測で gen_capture が参照する。
static C0_GPS: AtomicU32 = AtomicU32::new(0);
/// GPS PPS エッジの世代カウンタ。出力エッジ間に進んでいなければ GPS 欠落 = C0_GPS は古い → 補正に使わない。
static C0_GEN: AtomicU32 = AtomicU32::new(0);
/// 1 秒のPIO tick 数 (= clk_sys / PIO_CYCLES_PER_TICK)。位相を mod 1秒する用。
const TICKS_PER_SEC: i64 = 62_500_000; // 125MHz / 2

/// x を ±m/2 に正規化 (mod m で中心化)。位相を ±0.5秒に畳む用。
fn signed_mod(x: i64, m: i64) -> i64 {
    let r = x % m;
    if r > m / 2 {
        r - m
    } else if r < -m / 2 {
        r + m
    } else {
        r
    }
}

/// gen_capture_task 用の高優先度割込エグゼキュータ。ループバックエッジ→Instant 読みの
/// ウェイクアップ遅延 (thread executor だと UART 処理等で ~ms) を ~µs に下げ、位相測定を精密化する。
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SWI_IRQ_0() {
    EXECUTOR_HIGH.on_interrupt()
}

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
        C0_GPS.store(x, Ordering::Relaxed); // stage②: 最新 GPS エッジの生カウンタを共有
        C0_GEN.fetch_add(1, Ordering::Relaxed); // 世代++ (gen_capture が欠落検出に使う)
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
async fn time_task() {
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
    }
}

/// 規律 PPS 出力の周期管理 + ループバック計測 (GP3 生成 / GP4 捕捉)。**エッジに同期して**
/// 1 出力エッジにつき 1 回だけ次周期を更新する (= freq 規律 + 位相同期を 1 箇所で・1 サンプル遅れで)。
/// time_task の 1Hz タイマと非同期に補正すると位相が発振したので、ここに集約した。
/// ※ freq/位相規律はループバック (GP3→GP4 ジャンパ) 接続が前提。未接続だと初期周期で自走する。
#[embassy_executor::task]
async fn gen_capture_task(
    mut sm_gen: StateMachine<'static, PIO0, 1>,
    mut sm: StateMachine<'static, PIO0, 2>,
    k: u32, // stage② 較正済みカウンタオフセット (C0−C2)
) {
    let clk = clk_sys_freq() as i64;
    let ns_per_tick: i64 = PIO_CYCLES_PER_TICK as i64 * 1_000_000_000 / clk;
    let mut last_x: Option<u32> = None;
    let mut count: u32 = 0;
    let mut last_gen: u32 = 0;
    let mut lock_cnt: u32 = 0; // 連続で位相が小さかったエッジ数 (≥LOCK_HOLD でロック)
    let mut reject_cnt: u32 = 0; // 連続で外れ値棄却したエッジ数
    let mut ppb_trim: i64 = 0; // 位相 I 項 → 周波数トリム。**milli-ppb 単位** (高分解能。整数 ppb だと
                               // ctrl/DEN の truncate で deadzone、周期も 8ppb 量子化でリミットサイクルした)。
    let mut frac_acc: i64 = 0; // 周期の小数端数 (sigma-delta dither, ×1e12 スケール)。sub-cycle 周波数分解能。
    let mut last_ctrl: i64 = 0; // 前エッジの予測位相 (D 項の微分用)
    let mut last_pd: i64 = 0; // 前エッジの P+D 補正 (Smith 予測の在飛行分。d≈2 なので 1 エッジ分)
    loop {
        let x = sm.rx().wait_pull().await; // x = 出力エッジの生カウンタ (C2_out)
        // GPS PPS の世代。前回の出力エッジから進んでいなければ GPS 欠落 → C0_GPS が古い → 補正に使わない
        // (弱信号で PPS が時々落ちると、古い基準で巨大補正が走り出力が ~100ms 飛ぶのを防ぐ。断中はホールド)。
        let gen = C0_GEN.load(Ordering::Relaxed);
        let fresh = gen != last_gen;
        last_gen = gen;
        // 出力周期。PIO ~68s 周回グリッチの偽エッジは間隔が異常 → 位相補正に使わない。
        let interval_ns = last_x.map(|lx| lx.wrapping_sub(x) as i64 * ns_per_tick);
        let sane = interval_ns.is_some_and(|iv| (iv - 1_000_000_000).abs() < 300_000_000);
        // stage② PIO ハード位相: 出力エッジ と 最新 GPS エッジ の生カウンタ差 (mod 1秒) を ns に。
        // Instant を介さないので executor のウェイクアップ遅延 (~ms) に汚されず 16ns 分解能。
        let c0 = C0_GPS.load(Ordering::Relaxed);
        let elapsed = c0.wrapping_sub(x).wrapping_sub(k) as i32 as i64; // GPS→出力 の経過 tick
        let hwphase_ns = signed_mod(elapsed, TICKS_PER_SEC) * ns_per_tick;
        // 旧手法 (比較用 emit のみ): 出力エッジの UTC 時刻 (Instant 経由) → 秒境界からのズレ。
        let (ppb, phase) = CLOCK.lock(|c| {
            let c = c.borrow();
            (c.freq_ppb(), c.now_from_instant_ns(now_local_ns()).map(snap_to_second_ns))
        });
        // 制御に使う位相: PIO ハード(stage②)。PHASE_USE_HW=false で旧 Instant 測定に切替 (比較計測用)。
        let ctrl = if PHASE_USE_HW { hwphase_ns } else { phase.unwrap_or(0) };
        // 実験モード: 制御構成を ~130 エッジ毎に巡回し実機で比較 (0=P,1=PD,2=PI,3=PID,4=PID+Smith)。
        // 先頭を PID+Smith にしてコールドスタートをロックしてから P/PD/PI/PID を回す。本番は常に 4。
        let cfg: u32 = if PHASE_EXPERIMENT {
            let ph = (count / 130) % 5;
            if ph == 0 { 4 } else { ph - 1 }
        } else {
            4
        };
        let use_i = matches!(cfg, 2 | 3 | 4);
        let use_d = matches!(cfg, 1 | 3 | 4);
        let use_smith = cfg == 4;
        // **Smith 予測子**: 在飛行中(前エッジに出した、まだ位相に現れてない) P/D 補正を引いた予測位相 pred で
        // 制御 → ループ遅れ d≈2 を補償し ζ 公式が成立。Smith 無しの構成では生の ctrl を使う。
        let pred = if use_smith { ctrl - last_pd } else { ctrl };
        // valid = この位相サンプルが信用できるか (GPS 新鮮 + 間隔健全)。
        let valid = fresh && sane && c0 != 0;
        let mut p_corr: i64 = 0; // 位相 P 項 (ns)。速い過渡を吸収。
        let mut d_corr: i64 = 0; // 位相 D 項 (ns)。位相速度に比例し振動を減衰。
        if valid {
            let locked = lock_cnt >= LOCK_HOLD;
            if locked && ctrl.abs() > OUTLIER_NS && reject_cnt < OUTLIER_MAX {
                reject_cnt += 1; // ロック中の単発 garbage → 補正せずホールド (出力を飛ばさない)
            } else {
                reject_cnt = 0;
                // I 項: ロック中、予測位相を周波数トリム(milli-ppb)に積分 → ドループを 0 に。P 実験は 0。
                if use_i {
                    if locked {
                        ppb_trim = (ppb_trim - pred * 1000 / PHASE_I_DEN)
                            .clamp(-PPB_TRIM_MAX * 1000, PPB_TRIM_MAX * 1000);
                    }
                } else {
                    ppb_trim = 0;
                }
                // P 項: 予測位相に比例。Smith で遅延補償済なので ζ=Kp/(2√Ki)=(1/8)/(2/√128)≈0.71 が効く。
                if pred.abs() > PHASE_DEADBAND_NS {
                    p_corr = (pred / 8).clamp(-100_000_000, 100_000_000);
                }
                // D 項: 予測位相の速度に比例し振動を減衰。
                if use_d && locked {
                    d_corr = ((pred - last_ctrl) / PHASE_D_DEN).clamp(-100_000_000, 100_000_000);
                }
                lock_cnt = if ctrl.abs() < LOCK_NS { (lock_cnt + 1).min(LOCK_HOLD) } else { 0 };
            }
        } // !valid (GPS 欠落等) は freq+I のみで自走 (holdover ホールド)、lock 状態は据え置き
        last_ctrl = pred;
        last_pd = p_corr + d_corr;
        // 周期 = clk − overhead + 周波数調整(整数 cycle) − P − D。**周波数調整を sigma-delta で小数 dither**:
        // clk*(ppb + ppb_trim/1000)/1e9 を ×1e12 スケールで累積し整数 cycle を切り出す。これで周期が
        // 1 clk cycle(=8ppb) 量子化に制約されず sub-8ppb 分解能になり、I 項が正確な周波数に整定=リミットサイクル消滅。
        frac_acc += clk * (ppb * 1000 + ppb_trim);
        let freq_cycles = frac_acc.div_euclid(1_000_000_000_000);
        frac_acc = frac_acc.rem_euclid(1_000_000_000_000);
        let period =
            clk - GEN_OVERHEAD + freq_cycles - (p_corr + d_corr) * clk / 1_000_000_000;
        let _ = sm_gen.tx().try_push(period as u32);
        if let Some(iv) = interval_ns {
            count += 1;
            // 制御信号を全部出力 → ホストでプラント同定 + ゲイン掃引シミュ + 項別比較ができる。
            // (互換のため既存 5 項 count/interval/dev/phase/hwphase を先頭に残し、cfg/trim/p/d を追記)
            info!(
                "PPSGEN count={} interval_ns={} dev_ns={} phase_ns={} hwphase_ns={} trim_ppb={} cfg={} p_ns={} d_ns={}",
                count, iv, iv - 1_000_000_000, phase.unwrap_or(0), hwphase_ns, ppb_trim / 1000, cfg, p_corr, d_corr
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
        "    jmp pin rising", // pin high → 立ち上がり (本物) → 捕捉
        "    jmp x-- low",    // X--; X≠0 なら low へ。X=0 のときだけ下へ落ちる
        "    jmp low",        // X=0 (周回) → 偽キャプチャせず継続 (~68s 毎の glitch を源で除去)
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
    let lb_pin = common.make_pio_pin(p.PIN_4);
    sm0.set_pin_dirs(PioDirection::In, &[&pps_pin]);
    sm2.set_pin_dirs(PioDirection::In, &[&lb_pin]);

    // SM0: GP2 (GPS PPS) を捕捉。
    let mut cfg_gp2 = PioConfig::default();
    cfg_gp2.use_program(&loaded, &[]);
    cfg_gp2.set_jmp_pin(&pps_pin);
    sm0.set_config(&cfg_gp2);
    sm0.set_enable(true);

    // stage②: SM2 を一旦 GP2 に向け、SM0 と同じ GPS エッジを両方で捕捉して生カウンタ差 K=C0−C2 を較正。
    // 両 SM は同じ clk_sys なので K は定数。set_config でピンを変えても scratch X は保たれるので K は有効。
    sm2.set_config(&cfg_gp2);
    sm2.set_enable(true);
    let mut k: u32 = 0;
    {
        let (mut ksum, mut kn): (i64, i64) = (0, 0);
        for i in 0..7u32 {
            // PPS は fix 後のみ出る。各エッジ最大 30s 待ち、出なければ較正打ち切り (hw 位相無効)。
            match with_timeout(Duration::from_secs(30), async {
                let c0 = sm0.rx().wait_pull().await;
                let c2 = sm2.rx().wait_pull().await;
                (c0, c2)
            })
            .await
            {
                Ok((c0, c2)) if i >= 2 => {
                    ksum += c0.wrapping_sub(c2) as i32 as i64;
                    kn += 1;
                }
                Ok(_) => {}
                Err(_) => break,
            }
        }
        if kn > 0 {
            k = (ksum / kn) as i32 as u32;
            info!("PHASE_K calibrated k={} (n={})", k as i32, kn);
        } else {
            warn!("PHASE_K calibration failed (no PPS)");
        }
    }

    // SM2 を GP4 (ループバック) に切替。set_config は X を触らないので K は有効のまま。
    let mut cfg_gp4 = PioConfig::default();
    cfg_gp4.use_program(&loaded, &[]);
    cfg_gp4.set_jmp_pin(&lb_pin);
    sm2.set_config(&cfg_gp4);

    // SM1: 規律 PPS 生成を GP3 へ。pull noblock で周期を保持し、毎周 X サイクル low + 短い high。
    // 周期は X に保持。カウントダウンは Y ← X を delay で潰すと 2 発目以降が壊れる (ハマった)。
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

    // pps_task と gen_capture を高優先度割込エグゼキュータで起動 (ウェイクアップ遅延 ms→µs)。
    interrupt::SWI_IRQ_0.set_priority(Priority::P0);
    let spawner_high = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_0);
    spawner_high.spawn(pps_task(sm0).unwrap());
    spawner.spawn(time_task().unwrap());
    spawner_high.spawn(gen_capture_task(sm1, sm2, k).unwrap());

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
            if s.starts_with("$PMTK705") {
                info!("FW {=str}", s);
            }

            // RMC (日付+時刻) と直近 PPS エッジを対応付けて UTC エポックを固定する。
            if s.get(3..6) == Some("RMC") {
                let time = s.split(',').nth(1).and_then(parse_hhmmss);
                let date = s.split(',').nth(9).and_then(parse_ddmmyy);
                // 新鮮な (未消費の) PPS エッジがある時だけペアする。PPS 欠落中は stale エッジを
                // 同じ秒に何度もペアして err が ±整数秒の偽値になるため、fresh==true を条件にする。
                if let (Some((h, mi, se)), Some((d, mo, y)), Some((pio_ns, inst_ns)), true) =
                    (time, date, pending, pending_fresh)
                {
                    pending_fresh = false; // このエッジは消費した
                    let unix_s = civil_to_unix(y as i64, mo as i64, d as i64, h as i64, mi as i64, se as i64);
                    let target = unix_s * 1_000_000_000;
                    let (ppb, err_ns, hold_ms) = CLOCK.lock(|c| {
                        let mut c = c.borrow_mut();
                        // 補正後の時刻精度: このエッジの UTC を「更新前のクロック (前回エポック+周波数)」で
                        // 予測し、実際の UTC 秒との差を取る = holdover 後の残差。PIO 時刻 (ns 精度) で計算。
                        // PPS が複数秒途切れて復帰すると整数秒ズレるので最寄り秒へ snap し、真の sub 秒残差を出す。
                        let err = c.now_ns(pio_ns).map(|pred| snap_to_second_ns(pred - target)).unwrap_or(0);
                        // この sync が前回 sync から何 ms 経っているか (= この err が何秒 holdover の誤差か)。
                        let hold = (c.holdover_ns(inst_ns) / 1_000_000) as u32;
                        c.update_epoch(pio_ns, inst_ns, target);
                        (c.freq_ppb(), err, hold)
                    });
                    // SYNC: err_ns は補正後の予測残差 (ns)、holdover_ms はこの err の holdover 経過。
                    info!(
                        "SYNC pps_local_us={} unix_s={} drift_us={} err_ns={} holdover_ms={}",
                        pio_ns / 1000,
                        unix_s,
                        ppb / 1000,
                        err_ns,
                        hold_ms
                    );
                }
            }
        }
    }
}
