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
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::interrupt;
use embassy_rp::interrupt::{InterruptExt, Priority};
use embassy_rp::peripherals::{PIO0, UART0};
use embassy_rp::pio::{
    Config as PioConfig, Direction as PioDirection, InterruptHandler as PioInterruptHandler, Pio,
    StateMachine,
};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config as UartConfig};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::Read;
use portable_atomic::{AtomicU32, Ordering};
use static_cell::StaticCell;

use gnssdo::{
    AlphaBetaBoost, ControlInput, Controller, IntegralRework, PhaseController, PhaseLockLoop,
    PhaseLockLoopConfig, PpsEvent, snap_to_second_ns,
};
use rp_pps::embassy::{SteeredPpsOutput, TimedPpsCapture};
use rp_pps::{NmeaLineAssembler, PpsGpsdo, PpsSteer};

/// 受信機固有レイヤ (MT3333/GYSFFMANC の PMTK 設定・ボーレート)。別受信機ならここだけ差し替え。
mod mt3333;

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

/// GPSDO 状態 (rp-pps の一体型 state bundle = gnssdo 規律 + PPS↔NMEA 対応付け)。pps_task が
/// `on_pps_edge` で周波数規律 + エッジ記録、main が `feed_nmea` で UTC エポック確定、time_task が読む。
/// 分類/Locked のみ規律/復帰 quarantine、NMEA ペアリングと fresh-once、残差診断は PpsGpsdo が内包する。
static CLOCK: BlockingMutex<CriticalSectionRawMutex, RefCell<PpsGpsdo>> =
    BlockingMutex::new(RefCell::new(PpsGpsdo::new()));

/// 位相制御の測定源。true=PIO ハード位相 (stage②, 本番)。false=旧 Instant 測定 (比較計測用)。
const PHASE_USE_HW: bool = true;
// 位相ループの gains は gnssdo の PhaseLockLoopConfig::DEFAULT に移動した (σ≈35ns 実測チューニング)。
/// **2026-06 改修 (実験中)**: 出力位相 wander (±~600ns, 卓越周期 ~36s) は受信律速ではなく **i_den での
/// type-II ループの underdamped 共振** と判明した。根拠 (5 系統で確認): 実ログ4本 + scope 実測が
/// いずれも卓越周期 32-37s (= 2π√(i_den) edge、i_den=32 で 35.5edge=36s) かつ電力の 73-94% が 16-64s 帯、
/// 受信品質(HDOP)・温度と無相関、実 PhaseLockLoop の step 応答が 35.0s で ~24 回リンギング、反証検証
/// (aliasing/K再較正/量子化/適応スイッチ) を全て排除。受信機の位相ステップ/低周波擾乱がこの固有モードを
/// 叩いてリンギング増幅される。**白色位相ノイズでは励起されず** (2 つの独立 host sim が 11ns→~5-8ns に
/// 減衰、replay も 46ns) excitation はモデル外なので、**修正の振幅効果は実機が判定者**。
/// **結論 (de-confound 後)**: i_den は wander の**振幅を変えない**。振幅は**受信律速**である。
/// 同一受信レベルで揃えると (受信プロキシ = slope_mppb の sd、PLL 非依存の GPS 間隔ノイズ量で層別)、
/// i_den=128 と 512 の wander はほぼ同じ (良受信 ~50ns、悪受信 ~140ns、差は ±10-20% でノイズ内)。固定
/// i_den=512 でも受信で 50→140ns 動く。先の掃引で「32→198/128→73/512→53ns と緩めると減る」と見えたのは
/// **単一サイクル掃引内の受信差** (i_den=32 区間がたまたま 4× 悪受信、slope-sd 847 vs 171) で、層別で消える。
/// 機構: 受信機 PPS の wander はループのコーナーより低周波なので i_den によらず追従され、underdamped モードは
/// **周期 (2π√i_den) を形作る**だけで共振利得は穏やか (~1.5×) かつ i_den 非依存 → 振幅は受信が決める。
/// **sim+実機で反証済みの罠**: kp_inv 8→4 も D 強化も連続時間 ζ 式に反して **不安定化** (Smith 1-edge 遅延 +
/// 整数切り捨て、kp_inv=4 は ζ<0 発散)。減衰ノブは触らない。
/// よって**ファームの位相ループでは振幅を削れない** (数十ns は良受信なら達成済み、悪受信での膨張は受信側
/// =アンテナ設置/タイミング受信機+qErr でしか安定化できない)。i_den は既定 128 へ戻す (512 の根拠は消えた)。
/// 将来のファーム余地: slope_mppb (間隔ジッタ) は GST と違い受信劣化を示すので、これで holdover を gating
/// する案 (悪受信中は出力を水晶外挿に倒し汚い PPS を追わない) は未検証だが筋がある。
/// (以下は適応ゲインの旧解説。適応は無効化。)
/// 位相ループ積分の分母。**適応ゲイン**: 落ち着き時 (|pred| < CALM_NS) は I_DEN_CALM で緩い温度ドリフトを
/// 締めて追従、外乱時 (ドライヤー級ショック) は I_DEN_CALM<<DISTURBED_SHIFT へ鈍化して windup/overshoot/
/// lock喪失を回避。calm と disturbed を decouple できる。
///
/// 実機チューニング: 固定32 は steady ns で良いがショックで ~7.6µs+lock喪失、固定16 はさらに messy。適応で
/// 「緩い=32 / ショック=128」の両取りを狙う (ショックの恩恵は実機が判定者、モデルは高ゲイン不信)。
///
/// 温度ランプ下の offset② 瞬時 wander を下げようと calm を 32→20 に締める案を試したが **HW で却下**: host
/// 閉ループ sweep (thermal curvature) は i_den=32→20 で位相 sd 114→72ns・発振なしを予測し、実機 calm でも
/// hwphase は ±50-80ns に締まった。しかし同一 |dppb/dt| で揃えると **ロック保持が悪化** (ランプ 0.5-1ppb/s で
/// lk 83%→48%)。高い積分ゲインがランプ中にオーバーシュートして unlock 窓を超えるため。GPSDO はロック保持が
/// 最優先なので 32 を維持する。sim の予測した headroom は実機のロック判定/外乱性質で実現しなかった
/// (host モデルはロック窓を持たないので高ゲインの不安定を過小評価する。gnssdo/tests/thermal_plant.rs の
/// i_den_sweep_under_thermal_curvature 参照)。残る温度 wander は環境 (連続 ±250ppb 振動) 律速で、firmware
/// ゲインでは安全に削れない。
const I_DEN_CALM: i64 = 512; // 実験: critically-damp 用 joint tuning (i_den 128→512 + D_DEN_TUNED 4→16, kp_inv=8 据置)。
/// 実験: critically-damped joint tuning の D。wander はループ自身の周期 (93s≈2π√128) でリンギングする
/// underdamped 共振 (hwphase 残差の autocorr +0.99/-0.54、短期床 12ns の 7.7×) と判明。個別ノブは反転する
/// (kp_inv↓/D↑ 単独は発振、i_den 単独は共振周波数を動かすだけ) が、**i_den↑ と D 弱化を joint で**動かすと
/// 閉ループ極が単位円の深部へ入り overdamped 化 (host: 300ns step で overshoot 23%→0%・リンギング 24→0回・
/// 整定 39→13 edge、ノイズ sd 7→2.5ns、100% lock)。共振が消えれば hwphase が ~92ns → 数十ns/床へ落ちる
/// はず。**host モデルは白色ノイズで共振を過小励起する**ので減衰特性の証拠どまり、実機ループバックの
/// hwphase 実測が判定者。effective だが Default は不変 (実験は firmware tuning のみ)。
const D_DEN_TUNED: i64 = 16;
const CALM_NS: i64 = 1_000;
const DISTURBED_SHIFT: u32 = 0; // 適応無効: 固定 i_den=128。旧 2 (32<<2=128) の calm/disturbed 切替を撤去

/// 実験 harness: **制御器ライブラリの同一 boot 巡回**。複数の出力位相制御手法 (gnssdo::Controller) を 1 boot で
/// 順に切替え、各セグメントに PRBS を注入すれば、受信を揃えた状態で手法差を同定できる (計測トラップ対策。
/// hwphase = 出力 vs 同じ受信機なので、固定比較や別 boot 比較では受信交絡が手法差に混ざる)。切替は
/// start_segment で出力周波数を連続にし観測器/Smith/lock を blank = 公平比較。**本番は false** (固定の本番制御器)。
const CONTROLLER_SWEEP: bool = false; // 制御器を同一 boot 巡回する比較実験。**production 既定 false** (固定 slot0)。true で巡回。
const CONTROLLER_SWEEP_EDGES: u32 = 300; // セグメント長 (locked エッジ)。interleave 用に短く = 各手法を多時刻サンプル
const CONTROLLER_LIST_LEN: usize = 5;
/// **interleave スケジュール** (de-confound)。固定順巡回だと「どの手法か」が時刻に 1:1 で写り、温度/受信ドリフト
/// と分離できない。各 cidx を**異なる時刻・順**で複数回サンプルすれば順序↔時刻交絡を破れる。各ラウンドを巡回シフト
/// した Latin-square 的並び (5 手法 × 5 ラウンド = 各手法が 5 つの異なる時間帯に出る)。全 25 後に繰り返す。
/// analyze_prbs.py は cidx で pool するので、同 cidx の複数セグメントが自動的に束ねられ誤差棒が得られる。
const SWEEP_SCHEDULE: [usize; 25] = [
    0, 1, 2, 3, 4, //
    2, 3, 4, 0, 1, //
    4, 0, 1, 2, 3, //
    1, 2, 3, 4, 0, //
    3, 4, 0, 1, 2, //
];
/// idx 番目の制御手法を生成 (巡回・本番初期化の単一の出所)。slot 0 = 現行 firmware の本番チューニングを保つ。
fn make_controller(idx: usize) -> Controller {
    match idx % CONTROLLER_LIST_LEN {
        // 本番: 現行 firmware の PID+Smith チューニング (構造変更で挙動を変えないよう保持)。
        0 => Controller::Pll(PhaseLockLoop::with_config(PhaseLockLoopConfig {
            i_den: I_DEN_CALM,
            calm_ns: CALM_NS,
            i_den_disturbed_shift: DISTURBED_SHIFT,
            d_den: D_DEN_TUNED,
            ..PhaseLockLoopConfig::DEFAULT
        })),
        // 本命候補: 固定ゲイン α-β 観測器 + 残差トリガ過渡 boost。
        1 => Controller::AbBoost(AlphaBetaBoost::new()),
        // フラグ無し連続積分 (I-enable ゲート廃止 + 積分入力クランプ + back-calc)。
        2 => Controller::IntegRework(IntegralRework::new()),
        // ライブラリ既定 PID+Smith (i_den=128/d_den=4。report の結論。比較用)。
        3 => Controller::Pll(PhaseLockLoop::new()),
        // 素朴 PID (Smith 無し。過去レポートの型 II)。
        _ => Controller::Pll(PhaseLockLoop::with_config(PhaseLockLoopConfig::NAIVE_PID)),
    }
}

/// 受信品質テレメトリ: GPS 間隔ジッタ (直近 RX_JIT_WIN エッジの interval 範囲 = max−min) を hysteresis で
/// 良/悪受信に判定し、RECEPTION_BAD/JITTER として publish する (ログの rxbad/jit。制御の分岐には使わない)。
const RX_JIT_HI_NS: i64 = 120; // これ超で悪受信へ
const RX_JIT_LO_NS: i64 = 70; // これ未満で良受信へ戻る (hysteresis で chatter 防止)
const RX_JIT_WIN: usize = 8;
/// 実験: **PRBS 自己同定**。ロック中、出力位相に既知の擬似ランダム ±PRBS_AMP_NS を注入 (plant 入力外乱) し、
/// hwphase の応答を inj と相互相関すると、受信ノイズ (inj と非相関→平均で落ちる) を分離して**閉ループ伝達**
/// (共振/実効 damping/帯域) を受信交絡なしに同定できる。制御器を巡回しながら各々に注入すれば、手法ごとの
/// 復旧応答 h[k] (共振/リンギング) を受信非依存に比較でき、go/no-go の判定軸になる (codex 推奨)。
const PRBS_INJECT: bool = false; // 各手法に PRBS 注入し h[k] を受信非依存比較する実験。**production 既定 false**。
const PRBS_AMP_NS: i64 = 96; // ±96ns (16ns 捕捉量子化より十分大、lock 窓 1µs より十分小)
/// 実験: **boost 強制発火 / 遷移回復計測**。PRBS(±96ns)は線形域の定常共振 h[k] しか測れず、ab_boost の売りで
/// ある遷移回復 (boost) は発火しないため未評価のままになる。そこでロック中、KICK_PERIOD locked エッジ毎に出力
/// 位相へ +KICK_AMP_NS のステップ外乱を 1 発注入する。≥ outlier_ns(3000) なので各手法は数エッジ outlier-reject
/// した後に本物外乱として受理し、ab_boost はそこで boost を発火する。kick イベントに整列して手法ごとの整定
/// エッジ数を測れば、遷移回復を受信非依存に比較できる(kick は PRBS LFSR と無相関なので h[k] には平均で落ちる
/// が、回復中の大振幅は h[k] 推定を汚すので解析側で kick 後 ~20 エッジを除外する)。本番は false。
const KICK_INJECT: bool = false; // boost 強制発火/回復計測の実験。**production 既定 false**。
const KICK_AMP_NS: i64 = 8000; // outlier_ns(3000) 超の明確なステップ。reject 後受理で boost 発火、lock 窓外
const KICK_PERIOD: u32 = 150; // locked エッジ毎に 1 発(セグメント 300 に ~2 発、回復が収まる間隔)

/// 定期 K 再較正の cadence (locked 中の出力エッジ数 ≈ 秒)。実効 K の温度ドリフト (#5 後で ~5ns/min) を
/// 追従する。5分なら最大 ~25ns 振れ → 2桁ns 維持。短くすると holdover 中断が増える tradeoff。
const RECAL_EDGES: u32 = 300;
/// 再較正で取る (c0,c2) 同一エッジ対の数。先頭 1 個は GP4→GP2 切替トランジェントとして捨て、残りで K を測る。
const RECAL_SAMPLES: u32 = 6;
/// 再較正サンプルの旧 K 近傍ゲート (tick)。正しい同一エッジ対なら c0-c2 は旧 K から高々 ~数十 tick
/// (5分のドリフトは ~6tick) なので、これを超える対は spurious エッジとのミスペア (~±1秒) として捨てる。
const RECAL_SAMPLE_TICK_GATE: u32 = 64; // ≈1µs。実ドリフト ~19ns/min に対し十分広く、±1秒ミスペアは確実に弾く
/// 再較正で測った新 K は段差で適用せず、1 エッジあたり最大このtick数だけ slew して寄せる。実効 K は連続的に
/// 這うので補正も連続が自然。段差適用だと累積ドリフト(~6tick=96ns)を loop が急補正し D 項が跳ねて
/// offset② が ~30s 暴れる (実機確認)。slew で ramp 化すると loop は滑らかに追従する。dk≈6 なら ~3 エッジで適用。
const RECAL_K_SLEW_TICKS: i32 = 2;

/// 最新の GPS PPS エッジの生カウンタ値 (SM0)。stage② の PIO 位相計測で gen_capture が参照する。
static C0_GPS: AtomicU32 = AtomicU32::new(0);
/// GPS PPS エッジの世代カウンタ。出力エッジ間に進んでいなければ GPS 欠落 = C0_GPS は古い → 補正に使わない。
static C0_GEN: AtomicU32 = AtomicU32::new(0);

/// 受信適応帯域: 1=悪受信 (遅い/減衰ループ)、0=良受信 (速いループ)。pps_task が間隔ジッタから set、
/// gen_capture_task が読んで (i_den,d_den) を切替える。
static RECEPTION_BAD: AtomicU32 = AtomicU32::new(0);
/// 直近の GPS 間隔ジッタ (ns, ログ用)。
static RECEPTION_JITTER: AtomicU32 = AtomicU32::new(0);


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
async fn pps_task(mut capture: TimedPpsCapture<'static, PIO0, 0>) {
    let mut count: u32 = 0; // ログ用エッジ連番
    let mut iv_buf = [0i64; RX_JIT_WIN]; // 直近 Locked エッジの interval_ns (受信品質ジッタ用 ring buffer)
    let mut iv_cnt: usize = 0;

    loop {
        // wait_edge + timeline.observe は TimedPpsCapture に委譲。生カウンタは edge.raw で取れる。
        let edge = capture.next_edge().await;
        let edge_raw = edge.raw;
        // エポックのアンカー用に Instant を読む (µs ジッタは絶対オフセットのみに効く)。
        let query_ns = now_local_ns();

        // 周波数規律 (Locked のみ・復帰 quarantine) + 次の RMC 用にエッジ記録を PpsGpsdo に委譲。
        // PPS_TS signal は不要に: エッジは共有 state に記録され、main の feed_nmea が拾う。ここは log だけ。
        count += 1;
        let step = CLOCK.lock(|g| g.borrow_mut().on_pps_edge(edge, query_ns));
        // stage② 共有 C0 は **Locked エッジのみ** publish。spurious/Irregular な GP2 捕捉を loopback の
        // 基準にすると、ループがそれと対にして出力が誤った点へ寄り、しかも capture SM の余分捕捉で SM0/SM2
        // の実効 K が runtime ドリフトする (オシロで offset② が ~µs まで育ち boot でリセット、を実機確認)。
        if matches!(step.event, PpsEvent::Locked { .. }) {
            C0_GPS.store(edge_raw, Ordering::Relaxed);
            C0_GEN.fetch_add(1, Ordering::Release); // gen++ (Release: gen 進行で直前の C0_GPS store が見える)
            // 受信品質: Locked エッジの interval ジッタ (直近 RX_JIT_WIN の範囲 = max−min) を hysteresis で判定。
            // 水晶オフセット (~+2900ns) は共通なので範囲では相殺し、純粋なエッジ間ジッタ (= 受信劣化) を拾う。
            iv_buf[iv_cnt % RX_JIT_WIN] = edge.interval_ns as i64;
            iv_cnt += 1;
            if iv_cnt >= RX_JIT_WIN {
                let (mut lo, mut hi) = (i64::MAX, i64::MIN);
                for &v in &iv_buf {
                    lo = lo.min(v);
                    hi = hi.max(v);
                }
                let jit = hi - lo;
                RECEPTION_JITTER.store(jit.clamp(0, u32::MAX as i64) as u32, Ordering::Relaxed);
                let was_bad = RECEPTION_BAD.load(Ordering::Relaxed) != 0;
                let now_bad = if was_bad { jit > RX_JIT_LO_NS } else { jit > RX_JIT_HI_NS };
                RECEPTION_BAD.store(now_bad as u32, Ordering::Relaxed);
            }
        }
        let (state, missed): (&str, u32) = match step.event {
            PpsEvent::First => ("First", 0),
            PpsEvent::Locked { .. } => ("Locked", 0),
            PpsEvent::Irregular { missed, .. } => ("Irregular", missed),
            PpsEvent::NonMonotonic { .. } => ("NonMono", 0),
        };
        // freq= は規律した時だけ ok/sane/gate/quar、規律しなかったエッジは none。
        let freq = step.freq.map_or("none", |fu| fu.as_str());
        info!(
            "PPS count={} interval_us={} interval_ns={} state={=str} missed={} freq={}",
            count,
            edge.interval_ns / 1000,
            edge.interval_ns,
            state,
            missed,
            freq
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
        let (now, ppb, holdover, locked) = CLOCK.lock(|g| {
            let g = g.borrow();
            // 連続クエリは Instant 系 (サブ秒は µs 精度)。エポックの絶対対応は Instant アンカー。
            (
                g.now_from_query_ns(local),
                g.freq_ppb(),
                g.holdover_ns(local),
                g.frequency_locked(),
            )
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

/// 定期 K 再較正。両捕捉 SM (SM0=GP2 GPS, SM2=GP4 出力ループバック) の生カウンタは runtime/温度で相対
/// スリップし、boot 較正した K が古くなって offset② (出力 PPS vs GPS の絶対位相) がゆっくり這う (実機確認:
/// C0 publish を Locked 限定にした後も ~5ns/min・温度相関)。SM2 を一時的に GP2 へ向け、SM0 と同一 GP2
/// エッジで K=mean(c0-c2) を測り直して現在の相対関係に追従する。
///
/// SM2 が出力 (GP4) を見ない間も SM1 は毎秒 1 周期語を消費し、空 FIFO だとパルスを落とす (出力プログラムは
/// 前周期を保持しない)。そこで GP2 エッジ待ちをブロッキングにせず ~100ms 毎にポーリングし、その都度 `last_period`
/// を**生 push** して holdover を養う (dither は通常制御経路でのみ前進。満杯なら set_period は no-op)。GPS が
/// 数秒遅れても補充が止まらないので starvation しない。clean (spread≤2 tick・≥3 サンプル・各サンプルは旧 K
/// 近傍) かつ |Δk|≤cap のときだけ新 k を採用。
async fn recal_k(
    sm: &mut StateMachine<'static, PIO0, 2>,
    output: &mut SteeredPpsOutput<'static, PIO0, 1>,
    cfg_gp2: &PioConfig<'static, PIO0>,
    cfg_gp4: &PioConfig<'static, PIO0>,
    k: u32,
    last_period: u32,
) -> u32 {
    // SM2 を GP2 へ (jmp_pin だけ替わる同一 program。set_config は X を触らないので K の基準カウンタは不変)。
    sm.set_config(cfg_gp2);
    while sm.rx().try_pull().is_some() {} // 切替前後の stale/in-flight 捕捉を捨てる
    let mut samples: heapless::Vec<(u32, u32), 8> = heapless::Vec::new();
    for i in 0..RECAL_SAMPLES {
        // 同一エッジ対の鍵: c2 を待つ「前」に gen_before を確定する。pps_task は C0_GPS store 後に C0_GEN を
        // Release++ するので、この順なら c2 と同じ GP2 エッジの publish を待てる (c2 後に待ち始めると既に
        // publish 済みのとき次秒へずれ、1 秒ミスペアが安定しうる)。
        let gen_before = C0_GEN.load(Ordering::Acquire);
        // GP2 エッジを ~100ms 毎にポーリングしつつ毎回出力を生 push する。ブロッキング待ちだと GPS が遅れた
        // 数秒のあいだ補充が止まり FIFO が枯れるので、待つ間も必ず養う (満杯なら set_period は no-op)。
        let sample_deadline = Instant::now() + Duration::from_secs(3);
        let mut c2: Option<u32> = None;
        loop {
            let _ = output.output_mut().set_period(last_period);
            if let Some(v) = sm.rx().try_pull() {
                c2 = Some(v);
                break;
            }
            if Instant::now() >= sample_deadline {
                break;
            }
            Timer::after(Duration::from_millis(100)).await;
        }
        let Some(c2) = c2 else { break }; // GPS 欠落 → 中断 (旧 k 維持)
        // pps_task が同エッジを publish (gen 進行) するのを短時間待つ。進まなければこのサンプルは捨てる。
        let deadline = Instant::now() + Duration::from_millis(150);
        while C0_GEN.load(Ordering::Acquire) == gen_before && Instant::now() < deadline {
            embassy_futures::yield_now().await;
        }
        if C0_GEN.load(Ordering::Acquire) == gen_before {
            continue; // publish 無し → 捨てる
        }
        let c0 = C0_GPS.load(Ordering::Relaxed);
        // 先頭 (i==0) は GP4→GP2 切替トランジェントとして捨てる。
        if c0 == 0 || i == 0 {
            continue;
        }
        // サンプル毎の旧 K 近傍ゲート: 正しい対なら c0-c2 は旧 K から高々 ~数十 tick (5分のドリフトは ~6tick)。
        // SM2 が spurious な GP2 エッジを拾って別エッジと対になると ~±1秒 (≫ gate) ずれるので棄却する。
        let d = c0.wrapping_sub(c2) as i32;
        if d.wrapping_sub(k as i32).unsigned_abs() > RECAL_SAMPLE_TICK_GATE {
            continue;
        }
        let _ = samples.push((c0, c2));
    }
    // SM2 を GP4 へ戻す (X 保持)。drain して次の GP4 捕捉が新鮮な出力エッジになるように。
    sm.set_config(cfg_gp4);
    while sm.rx().try_pull().is_some() {}
    // 評価: spread ゲート → mean → |Δk| cap。怪しければ旧 k 維持 (誤った K で出力を飛ばさない)。
    if samples.len() < 3 {
        warn!("PHASE_K recal abort: too few samples n={}", samples.len());
        return k;
    }
    let (mut lo, mut hi) = (i32::MAX, i32::MIN);
    for (c0, c2) in samples.iter() {
        let d = c0.wrapping_sub(*c2) as i32;
        lo = lo.min(d);
        hi = hi.max(d);
    }
    let spread = hi as i64 - lo as i64;
    if spread > 2 {
        warn!("PHASE_K recal reject: spread={} n={}", spread, samples.len());
        return k;
    }
    let Some(nk) = rp_pps::calibrate_loopback_offset(samples.iter().copied()) else {
        return k;
    };
    let dk = nk.wrapping_sub(k) as i32;
    if dk.abs() > 500 {
        warn!(
            "PHASE_K recal reject: |dk|={} > 500 (k={} nk={})",
            dk, k as i32, nk as i32
        );
        return k;
    }
    info!(
        "PHASE_K recal OK k={} -> {} dk={} n={} spread={}",
        k as i32,
        nk as i32,
        dk,
        samples.len(),
        spread
    );
    nk
}

/// 規律 PPS 出力の周期管理 + ループバック計測 (GP3 生成 / GP4 捕捉)。**エッジに同期して**
/// 1 出力エッジにつき 1 回だけ次周期を更新する (= freq 規律 + 位相同期を 1 箇所で・1 サンプル遅れで)。
/// time_task の 1Hz タイマと非同期に補正すると位相が発振したので、ここに集約した。
/// ※ freq/位相規律はループバック (GP3→GP4 ジャンパ) 接続が前提。未接続だと初期周期で自走する。
#[embassy_executor::task]
async fn gen_capture_task(
    mut output: SteeredPpsOutput<'static, PIO0, 1>,
    mut sm: StateMachine<'static, PIO0, 2>,
    mut k: u32, // stage② 較正済みカウンタオフセット (C0−C2)。定期再較正で更新する。
    cfg_gp2: PioConfig<'static, PIO0>, // SM2 を GPS(GP2) へ向ける config (再較正用)
    cfg_gp4: PioConfig<'static, PIO0>, // SM2 を出力ループバック(GP4) へ向ける config (通常運用)
) {
    let clk = clk_sys_freq();
    let mut last_x: Option<u32> = None;
    let mut count: u32 = 0;
    let mut last_gen: u32 = 0;
    let mut k_target: u32 = k; // 再較正が測った最新 K。毎エッジ k をここへ slew して寄せる (段差→ramp)。
    // 直近の出力周期語。再較正中の holdover 生 push に再利用する。必ず set_next_period (下) で代入されてから
    // recal_k に渡る (再較正はループ末尾＝周期語確定後にしか発火しない) ので、初期値は不要。
    let mut last_period: u32;
    let mut edges_since_recal: u32 = 0;
    // 出力位相の制御器 (gnssdo::Controller)。sub-cycle period 生成 (dither) + period word push は
    // SteeredPpsOutput (rp-pps) が内包するので、ここは freq_mppb と phase_corr を渡すだけ。本番は
    // slot 0 (現行 firmware の PID+Smith チューニング)。CONTROLLER_SWEEP=true で locked 中に巡回する。
    let mut sweep_idx: usize = 0; // SWEEP_SCHEDULE 内の位置 (0..25)。実 cidx = SWEEP_SCHEDULE[sweep_idx]。
    let mut controller = make_controller(SWEEP_SCHEDULE[sweep_idx]);
    let mut edges_since_sweep: u32 = 0;
    let mut edges_since_kick: u32 = 0; // KICK 注入の locked エッジカウンタ
    // 制御器巡回の公平切替に使う、直近に適用した出力周波数 (= pred_mppb + trim_mppb)。次手法へ residual
    // trim = applied_freq − new_pred を渡し、出力周波数を連続にする (再捕捉を比較に混ぜない)。毎エッジ
    // controller.step 後に代入してから (同一反復の後段の) 切替で読むので、初期値は不要。
    let mut last_applied_freq_mppb: i64;
    // PRBS 自己同定の LFSR 状態 (16bit maximal-length, tap 0xB400)。
    let mut prbs_lfsr: u16 = 0xACE1;
    loop {
        let x0 = sm.rx().wait_pull().await; // 出力エッジの生カウンタ (C2_out)。wait_pull は FIFO 最古を返す。
        // backlog があれば最新エッジまで drain する (K 較正の drain と同じ対策)。ループが holdover 復帰等で
        // 遅れると stale な出力エッジが FIFO に溜まり、それを現 GPS と対にすると N 秒ずれる。loopback_phase の
        // 公称秒 fold はその N 秒を消すが N×ppm×1s が残り、hwphase を ~0 と誤認させて誤った位相に再ロックする
        // (オシロで実害 ~9.5µs=3×ppm×1s を確認)。最新エッジを使えば現 GPS と正しく対になる。
        let (x, dropped) = rp_pps::latest_capture(x0, || sm.rx().try_pull());
        if dropped > 0 {
            warn!("PPSGEN drained {} stale output edge(s) (loop lagged)", dropped);
        }
        // 出力は GPS より僅かに先行しうる。x[N] で起きた時点でまだ pps_task が GPS[N] を store して
        // いないと C0_GPS は GPS[N-1] のまま → x[N] を GPS[N-1] と対にして +ppm×1s (≈3.3µs) のスパイクに
        // なる。同秒の GPS エッジ (世代が進む) を短時間だけ協調待ち。pps_task と同 executor なので busy wait
        // 不可: yield_now で必ず手を放す。timeout 時は GPS 先行/欠落とみなし既存の fresh gating が判定。
        let start_gen = C0_GEN.load(Ordering::Acquire);
        // 既に今秒の GPS が来ていれば (前回反復から世代が進んでいれば) 待つ必要はない。出力が GPS を
        // 先行していて GPS[N] 未到着のときだけ、同秒 GPS を短時間協調待ち (GPS 先行/欠落時は timeout)。
        if start_gen == last_gen {
            let wait_until = Instant::now() + Duration::from_micros(500);
            while C0_GEN.load(Ordering::Acquire) == start_gen && Instant::now() < wait_until {
                embassy_futures::yield_now().await;
            }
        }
        // GPS PPS の世代。前回の出力エッジから進んでいなければ GPS 欠落 → C0_GPS が古い → 補正に使わない
        // (弱信号で PPS が時々落ちると、古い基準で巨大補正が走り出力が ~100ms 飛ぶのを防ぐ。断中はホールド)。
        let cur_gen = C0_GEN.load(Ordering::Acquire);
        // GPS 世代が前回からちょうど +1 のときだけ信用する。>1 はループ遅延/holdover 復帰で複数 GPS
        // エッジを跨いだ＝ペアリングが不確実なので invalid 扱い (1 サンプル捨てて次から正しく対にする)。
        let fresh = cur_gen.wrapping_sub(last_gen) == 1;
        last_gen = cur_gen;
        // 出力周期。PIO ~68s 周回グリッチの偽エッジは間隔が異常 → 位相補正に使わない。
        let interval_ns = last_x.map(|lx| rp_pps::interval_ns(lx, x, clk) as i64);
        let sane = interval_ns.is_some_and(|iv| (iv - 1_000_000_000).abs() < 300_000_000);
        // 再較正で測った K (k_target) へ毎エッジ最大 RECAL_K_SLEW_TICKS だけ slew。段差で適用すると
        // 累積ドリフトを loop が急補正し snap するので、ramp 化して滑らかに追従させる。
        if k != k_target {
            let diff = (k_target.wrapping_sub(k) as i32).clamp(-RECAL_K_SLEW_TICKS, RECAL_K_SLEW_TICKS);
            k = k.wrapping_add(diff as u32);
        }
        // stage② PIO ハード位相: 出力エッジ と 最新 GPS エッジ の生カウンタ差 (mod 1秒) を ns に。
        // Instant を介さないので executor のウェイクアップ遅延 (~ms) に汚されず 16ns 分解能。
        let c0 = C0_GPS.load(Ordering::Relaxed);
        let hwphase_ns = rp_pps::loopback_phase_ns(c0, x, k, clk);
        // ペアリング診断: fold 前の生 tick 差。正しい隣接エッジ対なら小さい(=真の位相)、ミスペア(欠落/
        // drain/再ロック後に非隣接エッジと対)だと ≈±1秒。fold がそれを隠し ppm×1s 残差にループがロック
        // する (9.5µs バグ族)。|raw_lag| が ~10ms 超なら gross ミスペアとして invalid 化し、loop に渡さない。
        // hwphase=0 ⟺ GP3=GPS を loopback 自身で(scope に頼らず)保つ。raw_lag はログして機構を観測。
        let raw_lag = rp_pps::loopback_raw_lag_ticks(c0, x, k);
        let max_lag = (clk / 200) as i32; // capture tick ≈ clk/2 なので clk/200 ≈ 10ms
        let paired = raw_lag.unsigned_abs() <= max_lag as u32;
        // 旧手法 (比較用 emit のみ): 出力エッジの UTC 時刻 (Instant 経由) → 秒境界からのズレ。
        let (pred_mppb, slope_mppb, phase) = CLOCK.lock(|g| {
            let g = g.borrow();
            (
                g.predicted_freq_mppb(),
                g.freq_slope_mppb(),
                g.now_from_query_ns(now_local_ns()).map(snap_to_second_ns),
            )
        });
        // 制御に使う位相: PIO ハード(stage②)。PHASE_USE_HW=false で旧 Instant 測定に切替 (比較計測用)。
        let ctrl = if PHASE_USE_HW {
            hwphase_ns
        } else {
            phase.unwrap_or(0)
        };
        // valid = この位相サンプルが信用できるか (GPS 新鮮 + 間隔健全 + 隣接エッジ対)。!valid は holdover
        // ホールド。paired=false (gross ミスペア) は fold が隠す ppm×1s 残差を避けるため捨てる。
        if !paired {
            warn!("PPSGEN mispair raw_lag={} ticks (>~10ms) -> drop", raw_lag);
        }
        let valid = fresh && sane && c0 != 0 && paired;
        // 選択中の制御器に位相誤差を渡す (gnssdo::Controller。本番=slot0、CONTROLLER_SWEEP 中は巡回)。
        let u = controller.step(ControlInput {
            err_ns: ctrl,
            valid,
        });
        // 周波数 = 水晶推定の 1 サンプル先予測 (mppb, ppb 丸めなし) + 制御器の周波数トリム (milli-ppb)。
        // 温度ランプ中、単一 EMA だと推定が遅れて出力位相が ramp 速度に比例してずれる (offset② のドリフト)。
        // DisciplinedClock の α-β が傾きを先回りするので、出力周期にその予測周波数を入れて定常誤差を抑える。
        // 位相補正は dither が cycle に変換して引く。
        let freq_mppb = pred_mppb + u.trim_mppb;
        last_applied_freq_mppb = freq_mppb; // 制御器巡回の公平切替用 (次手法へ residual trim を継ぐ)
        // PRBS 自己同定: ロック中、出力位相に既知の ±PRBS_AMP_NS 擬似ランダム外乱を注入(plant 入力)。ループは
        // これを hwphase 経由で見て打ち消すので、inj と hwphase の相互相関で閉ループ伝達(共振)が受信交絡なしに出る。
        let prbs_inj = if PRBS_INJECT && u.locked {
            let bit = prbs_lfsr & 1;
            prbs_lfsr = (prbs_lfsr >> 1) ^ if bit != 0 { 0xB400 } else { 0 };
            if bit != 0 { PRBS_AMP_NS } else { -PRBS_AMP_NS }
        } else {
            0
        };
        // boost 強制発火 / 回復計測: ロック中 KICK_PERIOD locked エッジ毎に出力位相へ +KICK_AMP_NS のステップ。
        // ≥ outlier_ns なので各手法は数エッジ reject 後に受理し、ab_boost はそこで boost 発火 → kick 整列で
        // 整定エッジ数を手法比較できる。kick イベントは kick_ns でログし、解析は kick 後窓を h[k] から除外する。
        let kick_inj = if KICK_INJECT && u.locked {
            edges_since_kick += 1;
            if edges_since_kick >= KICK_PERIOD {
                edges_since_kick = 0;
                KICK_AMP_NS
            } else {
                0
            }
        } else {
            0
        };
        // 周期語を保持: 再較正中の holdover 生 push に再利用 (dither はこの通常経路でのみ前進)。
        last_period = output.set_next_period(freq_mppb, u.pcorr_ns + prbs_inj + kick_inj);
        if let Some(iv) = interval_ns {
            count += 1;
            // 制御信号を全部出力 → ホストでプラント同定 + 制御器比較 + 項別比較ができる。
            // 互換のため既存 5 項 count/interval/dev/phase/hwphase を先頭に残す。cidx=制御器インデックス
            // (CONTROLLER_SWEEP の segment 鍵)、state=制御器の内部状態コード (ab_boost の boost 等)。
            info!(
                "PPSGEN count={} interval_ns={} dev_ns={} phase_ns={} hwphase_ns={} trim_ppb={} cidx={} p_ns={} d_ns={} trim_mppb={} slope_mppb={} raw_lag={} lk={} state={} jit={} rxbad={} inj_ns={} kick_ns={}",
                count,
                iv,
                iv - 1_000_000_000,
                phase.unwrap_or(0),
                hwphase_ns,
                u.trim_mppb / 1000,
                SWEEP_SCHEDULE[sweep_idx] as u32,
                u.dbg.p_ns,
                u.dbg.d_ns,
                u.trim_mppb,
                slope_mppb,
                raw_lag,
                u.locked as u8,
                u.dbg.state,
                RECEPTION_JITTER.load(Ordering::Relaxed),
                RECEPTION_BAD.load(Ordering::Relaxed),
                prbs_inj,
                kick_inj
            );
        }
        last_x = Some(x);
        // 定期 K 再較正: locked 中、RECAL_EDGES 出力エッジ毎に SM2 を GPS へ戻して実効 K を測り直す。
        // 実効 K は温度で這う (#5 後も ~5ns/min) ので boot 一発較正では追えない。再較正中は holdover。
        if u.locked {
            // 実験: 制御器巡回。locked エッジを数え、CONTROLLER_SWEEP_EDGES 毎に次の手法へ切替える (短い
            // unlock はカウントを進めないだけで segment を跨いで継続)。一定受信下で各手法の hwphase / PRBS
            // h[k] を比較する。切替は start_segment: 出力周波数が連続になるよう residual trim = 直近適用
            // 周波数 − 新手法の現 pred を継ぎ、観測器/Smith/lock を blank (再捕捉を比較に混ぜない=公平)。
            if CONTROLLER_SWEEP {
                edges_since_sweep += 1;
                if edges_since_sweep >= CONTROLLER_SWEEP_EDGES {
                    edges_since_sweep = 0;
                    edges_since_kick = 0; // 新セグメント先頭で kick 位相を揃える
                    sweep_idx = (sweep_idx + 1) % SWEEP_SCHEDULE.len();
                    let cidx = SWEEP_SCHEDULE[sweep_idx];
                    controller = make_controller(cidx);
                    controller.start_segment(gnssdo::ControlInit {
                        residual_trim_mppb: last_applied_freq_mppb - pred_mppb,
                    });
                    info!("CTRLSWEEP idx={} name={}", cidx as u32, controller.name());
                }
            }
            edges_since_recal += 1;
            if edges_since_recal >= RECAL_EDGES {
                edges_since_recal = 0;
                info!("PHASE_K recal start (after {} locked edges)", RECAL_EDGES);
                // 測定値は k_target へ。実際の k は通常ループで毎エッジ slew して寄せる (段差回避)。
                // dk cap は前回測定値 (k_target) 比で評価 (measured-to-measured の真ドリフト)。
                k_target = recal_k(&mut sm, &mut output, &cfg_gp2, &cfg_gp4, k_target, last_period).await;
                last_x = None; // GP4 捕捉が再較正の gap を跨ぐので次の interval は無効化
                last_gen = C0_GEN.load(Ordering::Acquire); // 復帰直後の偽 fresh ジャンプを避ける
            }
        } else {
            // cadence は「連続 locked」で数える。unlock したらリセットし、relock 直後に重い再較正
            // (SM2 を ~数秒奪う) が走らないようにする。
            edges_since_recal = 0;
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!(
        "pico-gnss: start (NMEA on UART0/GP1 @ {} baud, PPS on GP2 via PIO, GPSDO)",
        mt3333::GNSS_BAUD
    );

    // PPS on GP2 を PIO0 SM0 で rp-pps の PpsCapture でハードキャプチャ (~16ns 分解能)。
    let Pio {
        mut common,
        sm0,
        sm1,
        mut sm2,
        ..
    } = Pio::new(p.PIO0, Irqs);
    let mut capture = TimedPpsCapture::new(&mut common, sm0, p.PIN_2, clk_sys_freq());

    // stage②: SM2 (ループバック位相計測。rp-pps 外の実験機能) は capture と GP2 を共有して生カウンタ差
    // K=C0−C2 を較正する。embassy の set_jmp_pin は &Pin を要求し GP2 は一度しか make できないので、
    // capture.jmp_pin() で GP2 Pin を借り、capture プログラムをもう一枚 load する。
    let lb_loaded = common.load_program(&rp_pps::pps_capture_program());
    let lb_pin = common.make_pio_pin(p.PIN_4);
    sm2.set_pin_dirs(PioDirection::In, &[&lb_pin]);
    let mut cfg_gp2 = PioConfig::default();
    cfg_gp2.use_program(&lb_loaded, &[]);
    cfg_gp2.set_jmp_pin(capture.capture().jmp_pin()); // SM0 と同じ GP2 を捕捉 (fine tier へ escape)
    sm2.set_config(&cfg_gp2);
    sm2.set_enable(true);
    // K 較正: 較正前に両 SM の RX FIFO を drain → 次エッジから c0/c2 が同一 GP2 エッジになることを保証。
    // (古い値が残り c0/c2 が 1 エッジずれると、loopback の fold は公称秒 (62.5M tick) なので K に
    //  「実秒ぶんの tick」が混じり、mean が 水晶 ppm × 1s ≈ 数µs だけ boot ごとにずれる。)
    // 健全性検査: 同一エッジなら全 diff(c0−c2) が ~一定 (spread ≤ 1 tick)。混入があれば再 drain/retry。
    // PPS は fix 後のみ出るので各 30s 待ち、出なければ打ち切り。
    let mut k: u32 = 0;
    let mut k_n = 0usize;
    let mut k_clean = false;
    for attempt in 0..4u32 {
        while capture.capture_mut().try_read().is_some() {}
        while sm2.rx().try_pull().is_some() {}
        let mut samples: heapless::Vec<(u32, u32), 8> = heapless::Vec::new();
        for _ in 0..6u32 {
            match with_timeout(Duration::from_secs(30), async {
                let c0 = capture.capture_mut().wait_edge().await; // 較正は生カウンタ (fine tier へ escape)
                let c2 = sm2.rx().wait_pull().await;
                (c0, c2)
            })
            .await
            {
                Ok(pair) => {
                    let _ = samples.push(pair);
                }
                Err(_) => break,
            }
        }
        if samples.is_empty() {
            break; // PPS が来ない (no fix) → 打ち切り
        }
        let (mut lo, mut hi) = (i32::MAX, i32::MIN);
        for (c0, c2) in samples.iter() {
            let d = c0.wrapping_sub(*c2) as i32;
            lo = lo.min(d);
            hi = hi.max(d);
        }
        let spread = hi as i64 - lo as i64;
        if let Some(kk) = rp_pps::calibrate_loopback_offset(samples.iter().copied()) {
            k = kk; // 最良 (=最新) 候補。clean なら採用、不一致なら次 attempt で上書き。
            k_n = samples.len();
        }
        info!(
            "PHASE_K attempt={} n={} diff_lo={} diff_hi={} spread={} k={}",
            attempt,
            samples.len(),
            lo,
            hi,
            spread,
            k as i32
        );
        if samples.len() >= 3 && spread <= 1 {
            k_clean = true;
            break; // 同一エッジ確定
        }
        // 不一致 = 別エッジ混入 → 再 drain して取り直し
    }
    if k_clean {
        info!("PHASE_K calibrated k={} (n={})", k as i32, k_n);
    } else if k_n > 0 {
        warn!(
            "PHASE_K not clean (retries exhausted), using k={} (n={})",
            k as i32, k_n
        );
    } else {
        warn!("PHASE_K calibration failed (no PPS)");
    }
    // K 検証: SM2 を GP2 のまま数エッジ、loopback_phase が ~0 を確認 (数µs 出るなら K 不良)。
    while capture.capture_mut().try_read().is_some() {}
    while sm2.rx().try_pull().is_some() {}
    for _ in 0..3u32 {
        if let Ok((c0, c2)) = with_timeout(Duration::from_secs(30), async {
            let c0 = capture.capture_mut().wait_edge().await;
            let c2 = sm2.rx().wait_pull().await;
            (c0, c2)
        })
        .await
        {
            info!(
                "PHASE_K verify phase_ns={}",
                rp_pps::loopback_phase_ns(c0, c2, k, clk_sys_freq())
            );
        }
    }

    // SM2 を GP4 (ループバック) に切替。set_config は X を触らないので K は有効のまま。
    let mut cfg_gp4 = PioConfig::default();
    cfg_gp4.use_program(&lb_loaded, &[]);
    cfg_gp4.set_jmp_pin(&lb_pin);
    sm2.set_config(&cfg_gp4);

    // SM1: 規律 PPS 生成を GP3 へ (rp-pps の SteeredPpsOutput)。初期周期 = ppb=0 の 1Hz (内部で
    // output_period_cycles)。周期は gen_capture_task が毎エッジ set_next_period で操舵する
    // (dither+period word 計算は rp-pps、servo=PLL は firmware 側に残す)。high 幅 100ms = GPS
    // モジュール/GPSDO の一般的な 1PPS 幅 (外部機器へ配れる; 規律対象は立ち上がりエッジで幅に不依存)。
    const PPS_PULSE_NS: u32 = 100_000_000; // 100 ms
    let output = SteeredPpsOutput::new(&mut common, sm1, p.PIN_3, clk_sys_freq(), PPS_PULSE_NS);

    // pps_task と gen_capture を高優先度割込エグゼキュータで起動 (ウェイクアップ遅延 ms→µs)。
    interrupt::SWI_IRQ_0.set_priority(Priority::P0);
    let spawner_high = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_0);
    spawner_high.spawn(pps_task(capture).unwrap());
    spawner.spawn(time_task().unwrap());
    spawner_high.spawn(gen_capture_task(output, sm2, k, cfg_gp2, cfg_gp4).unwrap());

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
    loop {
        let n = match rx.read(&mut read_buf).await {
            Ok(n) => n,
            // Framing/Overrun は起動直後やモジュール再設定時に多発する。毎回 warn! すると
            // RTT が溢れて他のログ (ACK/NMEA) を trim してしまうので、黙って継続する
            // (assembler は次の '$' で再同期する)。
            Err(_) => continue,
        };

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

            // RMC を直近 PPS エッジ (pps_task が on_pps_edge で記録済み) と対応付けて UTC エポックを確定。
            // パース・fresh-once・残差診断は PpsGpsdo::feed_nmea が内包 (None=非RMC/fresh エッジ無し)。
            if let Some(r) = CLOCK.lock(|g| g.borrow_mut().feed_nmea(s)) {
                // SYNC: err_ns=補正後の予測残差 (timestamp 側)、fire_ns=逆予測残差 (fire_at_utc 側)、holdover_ms=err の holdover 経過。
                info!(
                    "SYNC pps_local_us={} unix_s={} drift_us={} err_ns={} fire_ns={} holdover_ms={}",
                    r.capture_ns / 1000,
                    r.unix_ns / 1_000_000_000,
                    r.freq_ppb / 1000,
                    r.err_ns,
                    r.fire_ns,
                    (r.holdover_ns / 1_000_000) as u32
                );
            }
        }
    }
}
