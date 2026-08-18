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
//! ソフト (embassy Input + Instant) はジッタが µs オーダの床 (M0+ の critical-section 全 IRQ マスク。
//! σ は負荷/boot で ~2〜10µs、衝突時は数十 µs のスパイク)。
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
use embassy_rp::gpio::{Input, Level, Output, Pull};
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
    ControlInput, Controller, OpenLoopFf, PhaseController, PhaseLockLoop, PhaseLockLoopConfig,
    PpsEvent, snap_to_second_ns,
};
use rp_pps::embassy::{SteeredPpsOutput, TimedPpsCapture};
use rp_pps::{NaivePeriodDither, NmeaLineAssembler, PpsGpsdo, PpsSteer, TimedEdge, fold_phase_ns};

/// 受信機固有レイヤ (MT3333/GYSFFMANC の PMTK 設定・ボーレート)。別受信機ならここだけ差し替え。
mod mt3333;

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
    // temp_task の free-running ADC→DMA 転送完了割込 (DMA_CH0 専用)。全 DMA ch は DMA_IRQ_0 を共有するが、
    // 本 firmware が使う DMA channel は temp_task の 1 本だけ (grep で他に DMA 使用なしを確認済み)。
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>;
});

/// GPSDO 状態 (rp-pps の一体型 state bundle = gnssdo 規律 + PPS↔NMEA 対応付け)。pps_task が
/// `on_pps_edge` で周波数規律 + エッジ記録、main が `feed_nmea` で UTC エポック確定、time_task が読む。
/// 分類/Locked のみ規律/復帰 quarantine、NMEA ペアリングと fresh-once、残差診断は PpsGpsdo が内包する。
static CLOCK: BlockingMutex<CriticalSectionRawMutex, RefCell<PpsGpsdo>> =
    BlockingMutex::new(RefCell::new(PpsGpsdo::new()));

/// **精度向上アークの単一ノブ (唯一の真実源)**。0..=5 を選ぶと派生ゲート (use_pio/apply_freq/
/// phase_servo/recal_on/temp_ff) が段を構成する。**5 = production (出荷既定)** で、現出荷ファームと
/// bit 一致することを下の const assert が機械保証する (計測 workflow が段ごとに一時変更する)。
/// - S0: naive 自走 (規律なし)。S1: soft 周波数規律 (+dither)。S2: 全 PIO I/O + loopback, 開ループFF。
/// - S3: PLL 閉ループ (Smith 内包)。S4: recal。S5: 温度FF = production。
const PRECISION_STAGE: u8 = 5;
// 上限ガード: stage>5 だと全ゲートが production 相当 (>=N) に倒れる一方、下の override-sentinel 強制は
// `!=5`/`==5` ゲートなので外れてしまい、override 付きで production 相当を焼けてしまう。それを禁止する。
const _: () = assert!(PRECISION_STAGE <= 5);
const fn use_pio(s: u8) -> bool {
    s >= 2
}
const fn apply_freq(s: u8) -> bool {
    s >= 1
}
const fn phase_servo(s: u8) -> bool {
    s >= 3
}
const fn recal_on(s: u8) -> bool {
    s >= 4
}
const fn temp_ff(s: u8) -> bool {
    s >= 5
}

/// substage override (センチネル=未指定→stage 既定)。S4b は recal off を巨大値で焼く。
/// **production では必ずセンチネル** (下の const assert が保証)。
const RECAL_EDGES_OVERRIDE: u32 = u32::MAX; // u32::MAX=未指定
const fn recal_eff(s: u8) -> u32 {
    if RECAL_EDGES_OVERRIDE != u32::MAX {
        RECAL_EDGES_OVERRIDE
    } else if recal_on(s) {
        RECAL_EDGES
    } else {
        u32::MAX // 実質 off (この閾値に達することはない)
    }
}

// production 不変の静的証明。const assert が通る =「PRECISION_STAGE=5 で焼けば現出荷ファームと
// 挙動同一」を機械保証する (cargo build が通れば成立)。ゲート梯子 (override 非依存) は全段で恒真。
const _: () = assert!(use_pio(5) && apply_freq(5) && phase_servo(5) && recal_on(5) && temp_ff(5));
const _: () = assert!(temp_ff(5) == TEMP_FF_ENABLE);
// override 依存の不変は **production (stage 5) のときだけ** 課す: stage 5 では override は必ず未指定
// (sentinel) で recal_eff は出荷リテラルに簡約。stage != 5 (計測) では override を許す。
const _: () = assert!(PRECISION_STAGE != 5 || RECAL_EDGES_OVERRIDE == u32::MAX);
const _: () = assert!(PRECISION_STAGE != 5 || recal_eff(5) == RECAL_EDGES);
// ラダー走行中 (stage != 5) は実験 harness と排他: 段切替えと別系統の実験フラグを同時に立てない。
const _: () = assert!(
    PRECISION_STAGE == 5 || !(CONTROLLER_SWEEP || PRBS_INJECT || KICK_INJECT || INTERMITTENT_EXP)
);

/// 非PIO 素朴経路 (S0/S1) の出力 1PPS の high 幅 (ms)。PIO 経路の `PPS_PULSE_NS` (100ms) と同じ
/// 慣習。立ち上がりエッジが規律対象で幅は時刻中立 (返り周期から引いて low 待ちにする)。
const NAIVE_PPS_PULSE_MS: u64 = 100;

/// 非PIO 素朴出力 (S0/S1) の立ち上がりエッジの**秒内位相** (ns, 0..1e9)。`naive_pps_out_task` が
/// 毎エッジ store し、`naive_pps_in_task` が GPS エッジの秒内位相と fold して soft hwphase を作る。
static OUT_SUBSEC_NS: AtomicU32 = AtomicU32::new(0);
/// 非PIO 素朴出力の世代カウンタ (軽量)。0 = まだ 1 度も出力していない (soft hwphase を載せない判定に使う)。
/// fold 指標は秒内なので ±1 秒の帰属誤りに不変 → C0_GEN 相当の同秒ペアリングは不要 (PLAN 採用#5)。
static OUT_GEN: AtomicU32 = AtomicU32::new(0);
/// 非PIO 素朴出力が**実際に**次エッジへ適用した dither 周期 (embassy tick)。`naive_pps_out_task` が
/// `NaivePeriodDither::next_ticks` の出力を毎エッジ store、`naive_pps_in_task` が PPSGEN に `dith_ticks`
/// として載せる。S1 はここが公称 (=tick_hz) を中心に sigma-delta で ±1 tick 揺れ、そのヒストグラムが
/// sub-tick 平均周波数を実機で行っている直接証跡になる。S0 (apply_freq=false) は公称固定 (= dither 不活性)。
static NAIVE_DITH_TICKS: AtomicU32 = AtomicU32::new(0);

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
const CONTROLLER_SWEEP: bool = false; // production: 固定 slot0 (i_den=512)。recal を効かせ ΔC スリップを bound する。
const CONTROLLER_SWEEP_EDGES: u32 = 1200; // セグメント長 (locked エッジ)。実験D(狭帯域+温度FF, 自然 wander): 最狭 2048 の周期 2π√2048≈285s に対し ≥4 周期/セグメント。wander 統計の安定化に長め。RECAL_EDGES(5000)より小。
const CONTROLLER_LIST_LEN: usize = 3;
/// **実験B: 3-way 帯域 (i_den) スイープの interleave スケジュール**。固定順巡回だと「どの帯域か」が時刻に
/// 1:1 で写り温度/受信ドリフトと分離できない。cidx 0/1/2 を各セグメントで巡回しつつ順序を散らすと、各帯域が
/// 複数の異なる時間帯に出て cidx↔時刻交絡を破れる。各 cidx 等回数 (9 セグメントで各 3 回)、全 9 後に繰り返す。
/// analyze_prbs.py は cidx で pool するので、同 cidx の複数セグメントが自動的に束ねられ誤差棒が得られる。
const SWEEP_SCHEDULE: [usize; 9] = [
    0, 1, 2, //
    1, 2, 0, //
    2, 0, 1, //
];
/// idx 番目の制御手法を生成 (巡回・本番初期化の単一の出所)。cidx 1 = 現行 firmware の本番帯域 (i_den=512)。

/// 共通ベース (適応無効・calm_ns 据置) に帯域 (i_den/d_den) と減衰形 (kp_inv/smith_edges) を載せる。
/// 実験B はこの i_den/d_den を 3 点振り (減衰形は現行 kp auto8/smith1 固定)、帯域そのものの効果を測る。
fn exp_cfg(i_den: i64, d_den: i64, kp_inv: i64, smith_edges: u32) -> PhaseLockLoopConfig {
    PhaseLockLoopConfig {
        i_den,
        calm_ns: CALM_NS,
        i_den_disturbed_shift: DISTURBED_SHIFT,
        d_den,
        kp_inv,
        smith_edges,
        ..PhaseLockLoopConfig::DEFAULT
    }
}

/// 実験D の 3-way 狭帯域グリッドの **effective** 制御器 config (make_controller と per-segment emit の唯一の真実源)。
/// `(i_den, d_den, kp_inv, smith_edges)`。減衰形は全 cidx で現行 production (kp_inv=auto8 / smith_edges=1)、
/// 比 i_den:d_den=32:1 を維持して帯域 (i_den) だけを 512/1024/2048 と狭側へ 3 点振る。kp_inv は exp_cfg(.., 0, 1)
/// で auto (kp_inv=0→PidSmith で実効 8) なので、emit には実効値 8 を載せる。
const CTRL_GRID: [(i64, i64, i64, u32); CONTROLLER_LIST_LEN] = [
    (I_DEN_CALM, D_DEN_TUNED, 8, 1), // cidx0: 現行 production (対照, i_den=512/d_den=16)
    (1024, 32, 8, 1),                // cidx1: 狭帯域 (i_den=1024/d_den=32)
    (2048, 64, 8, 1),                // cidx2: 最狭帯域 (i_den=2048/d_den=64)
];

fn make_controller(idx: usize) -> Controller {
    // **実験D: 狭帯域 (i_den) スイープ + 温度FF**。減衰形は現行 production (kp auto8/smith1) に固定し、帯域だけを
    // 3 点 (現行 512 / 狭 1024 / 最狭 2048) 振る。CONTROLLER_SWEEP で各 cidx を interleave しながら巡回し、自然
    // wander (hwphase std + 低周波共振帯) を帯域ごとに受信交絡なしで比較する (実験A で減衰形は wander を変えないと確定)。
    // CTRL_GRID が make_controller / per-segment emit の唯一の真実源。exp_cfg(i_den, d_den, 0, 1):
    // kp_inv=0=auto8 / smith=1 = 現行形。cidx0 (512/16/auto8/se1) は現行 production と bit 一致 (対照)。
    let (i_den, d_den, _kp_inv_emit, smith) = CTRL_GRID[idx % CONTROLLER_LIST_LEN];
    Controller::Pll(PhaseLockLoop::with_config(exp_cfg(i_den, d_den, 0, smith)))
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
const PRBS_INJECT: bool = false; // 実験D(狭帯域+温度FF): 自然 wander を測るので注入なし。**production 既定 false**。
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
/// 追従する。短くすると holdover 中断が増える tradeoff。
/// **実験A 退避**: recal の holdover gap (SM2 を数秒 GPS へ奪う) が PRBS h[k] 窓を汚すので、セグメント長
/// (CONTROLLER_SWEEP_EDGES=900) より十分大きくして実験中は recal を発火させない。実験後は production の
/// 300 へ戻す (boot 一発較正で K の初期値は確定済み)。
const RECAL_EDGES: u32 = 150; // production: ~2.5分毎に ΔC(実効K)再測。dk≈-5tick/300edge のスリップ sawtooth を ~40ns に抑える。

/// **実験: 間欠ループバック特性化**。ループバックを常時でなく間欠で使う設計 (普段は SM を別用途に空ける) に
/// したとき、出力位相がどう振る舞うかを実機で測る。閉窓 (通常の位相ループ + 窓入口で K 再較正) と
/// 開窓 (周波数FFのみ = `predicted_freq`、位相補正と recal を停止) を交互に回し、開窓中も hwphase は
/// 捕捉し続けてログ (適用しないだけ) → 開ループの位相ドリフトが hwphase と scope の両方に出る。
/// 開窓長 `EXP_TOFF_SCHEDULE` を巡回掃引し、「開窓時間 → 最大オフセット偏差」を得る。false で production。
const INTERMITTENT_EXP: bool = false;
const EXP_TON_EDGES: u32 = 30; // 閉窓 (locked edges≈秒): 入口の K 再較正 + ループ整定に十分
const EXP_TOFF_SCHEDULE: [u32; 4] = [30, 60, 120, 300]; // 開窓長 (locked edges≈秒) を巡回
/// 開窓の出力周波数: true=学習済みトリムを保持 (pred + 閉窓最後の trim。現実的な間欠設計) /
/// false=素朴 (pred のみ。ループ補正を丸ごと捨てる最悪ケース)。両者を比較して間欠化の特性差を測る。
const EXP_HOLD_TRIM: bool = true;

/// 温度フィードフォワード (gnssdo の増補型温度モデル) の runtime トグル。**実験A=false** (温度 wander の
/// 受信非依存 loop-shape を測る段階)。実験B/C で true にして温度FF の効果を測る。boot で
/// `CLOCK.set_temp_ff_enable` に渡す。
const TEMP_FF_ENABLE: bool = true; // 実験D(狭帯域+温度FF): 狭ループの追従遅れを die 温度の先行推定で埋める
/// 温度FF チューニング (実験D)。狭ループ (i_den≥1024, 遅れ>15s) では die 温度がループより進んだ周波数推定を
/// 与え、ループの追従遅れを埋めて wander を下げる。検証で 61s 平滑温度が水晶周波数を r=0.99 で予測すると判明。
/// 実験B の温度FF は sm_shift=4(16s)/gain_q8=64 と未チューニングだった。boot で set_temp_ff_enable(true) の直後に
/// CLOCK.set_temp_ff_params(TEMP_FF_SM_SHIFT, TEMP_FF_SHIFT, TEMP_FF_GAIN_Q8) で渡す。
const TEMP_FF_SM_SHIFT: u32 = 4; // ADC ノイズ除去の短 EMA。~16s 平滑 (finer 温度なら短めでもノイズが乗りにくい)
const TEMP_FF_SHIFT: u32 = 7; // 回帰 mean/cov の EMA (~128s)
const TEMP_FF_GAIN_Q8: i64 = 64; // matched-lead 経路では未使用 (旧 λ haircut。sweep 互換で保持)
/// matched-lead + 残差 observer の新ノブ (原理的 temp-FF)。die 温度→水晶温度の 1 次ラグ τ を構造的に補償
/// (Tcrys_hat = die_lp + (τ/Ts)·Δdie_lp)、非熱残差は固定ゲイン observer が吸う。λ haircut は廃止し full 権限。
const TEMP_FF_LAG_Q8: i64 = 1536; // τ/Ts を Q8 で (≈6s @1Hz)。検証で 61s 平滑が水晶を r≈0.99 で予測
const TEMP_FF_DLEAD_SHIFT: u32 = 2; // 微分の帯域制限 EMA (α=1/4)
const RESID_OBS_SHIFT: u32 = 2; // 残差 observer ゲイン K=1/2^shift=0.25
/// 再較正で取る (c0,c2) 同一エッジ対の数。先頭 1 個は GP4→GP2 切替トランジェントとして捨て、残りで K を測る。
const RECAL_SAMPLES: u32 = 6;
/// 再較正サンプルの旧 K 近傍ゲート (tick)。正しい同一エッジ対なら c0-c2 は旧 K から高々 ~数十 tick
/// (5分のドリフトは ~6tick) なので、これを超える対は spurious エッジとのミスペア (~±1秒) として捨てる。
const RECAL_SAMPLE_TICK_GATE: u32 = 64; // ≈1µs。実ドリフト ~19ns/min に対し十分広く、±1秒ミスペアは確実に弾く
/// 再較正で測った新 K は段差で適用せず、1 エッジあたり最大このtick数だけ slew して寄せる。実効 K は連続的に
/// 這うので補正も連続が自然。段差適用だと累積ドリフト(~6tick=96ns)を loop が急補正し D 項が跳ねて
/// offset② が ~30s 暴れる (実機確認)。slew で ramp 化すると loop は滑らかに追従する。dk≈6 なら ~3 エッジで適用。
const RECAL_K_SLEW_TICKS: i32 = 2;
/// 残留ドリフト調査 (20260704): stage-3 (recal 適用なし) で K を頻繁に測るが**適用しない** shadow recal。
/// 適用 K は起動時 K0 のまま (loop は stale K で pin が drift し続ける) で、測った K_shadow(t) を KSHADOW 行で
/// ログするだけ。判定: gap(t)=scope-hwphase が -(K_shadow(t)-K0) と一致すれば残ドリフト=実効 K ドリフト、
/// K_shadow 静止で gap だけ伸びれば出力生成/loopback の PLL 不可視バイアス (codex 助言の第1実験)。本番は false。
const SHADOW_RECAL: bool = false;
/// shadow K 測定の間隔 (locked 出力エッジ)。測定中は SM2 を GP2 へ奪う ~数秒 holdover になるので粗めに。
const SHADOW_EDGES: u32 = 60;
/// 残留ドリフト調査 (20260704): 出力周波数へ**ループに隠して**既知バイアスを step 注入する。生成 vs 参照の切り分け。
/// INJECT_START_COUNT 以降、freq_mppb に足すが loop の状態 (trim) には入れない。判定: (a) gap 傾きが注入分乗れば
/// 「生成は見えないバイアスを運べる」= 生成側容疑、(b) loop が trim で打ち消して gap 傾き不変なら「生成は clean、
/// 残ドリフトは参照/検出側」。1 ppb = 1000 mppb = 60 ns/min で残 ~+5 ns/min を明確に上回る。本番は 0。
const INJECT_FREQ_MPPB: i64 = 0;
/// 注入を on にする count (これ以前は baseline)。ロック整定後に段を作れるよう粗く 5 分相当。
const INJECT_START_COUNT: u32 = 300;
/// 残留ドリフト調査 (20260704): SM3 (純観測) を GP2 でなく GP4 (出力 loopback) へ向ける。
/// K ドリフトの残る容疑 = 捕捉パスの余分な通過 (偽捕捉) のピン間レート差、の検証。判定: 同じ GP4 を見る
/// c2−c3 が平坦なら march はピン固有イベント、c2−c3 が march すれば SM2 固有で仮説棄却。
/// 本番は false (c3=GP2 が KEXP の基準)。
const C3_WATCH_GP4: bool = false;
/// 残留ドリフト調査 (20260704, codex 助言): SM3 shadow — servo と K 測定の完全分離 + cadence sweep。
/// C3_WATCH_GP4=true で GP4 常駐の SM3 を、cadence 毎に GP2 へ SHADOW3_WINDOW_EDGES だけ向けて
/// K03=c0−c3 の同一エッジ対を測る (データは既存 KEXP 行に出る。SM2/servo は一切触らない)。
/// 判定: K03 の march が切替 1 回あたりに付くなら cadence 60→240 で傾きが 1/4 (切替起因の測定
/// アーティファクト = shadow-K march の正体)、時間に付くなら不変 (実ドリフト)。本番は false。
const SHADOW3: bool = false;
/// 前半の cadence (locked 出力エッジ数)。
const SHADOW3_EDGES_A: u32 = 60;
/// 後半の cadence。
const SHADOW3_EDGES_B: u32 = 240;
/// 後半 (cadence B) へ切り替える count。
const SHADOW3_PHASE_B_COUNT: u32 = 1500;
/// GP2 滞在エッジ数 (同一エッジ対のサンプル数。先頭 1 個は切替過渡として host 解析で捨てる)。
const SHADOW3_WINDOW_EDGES: u32 = 8;
// SHADOW3 は SM3 が GP4 常駐であることが前提。
const _: () = assert!(!SHADOW3 || C3_WATCH_GP4);
/// 温度 FF の長時間 A/B (20260705): 同一 boot 内で temp FF の適用だけを TEMPFF_ABAB_EDGES ごとに
/// on/off 交互に切り替え、自然室温での遅いふらつき σ への効果を交絡なし (同 boot、同受信、同構成) で測る。
/// 学習 (update_temp) は off 区間も回り続け、適用 (predicted_freq への反映) だけが切り替わる。
/// 区間境界は TFFAB 行でログし、host 解析が区間ごとに σ を出す。基盤構成は recal なし (stage-3) +
/// 幅合わせ + 一周コスト対称を想定。本番は false。
const TEMPFF_ABAB: bool = false;
/// 1 区間の長さ (locked 出力エッジ ≈ 秒)。30 分。遅いふらつきの周期 (64〜300 秒) より十分長く取る。
const TEMPFF_ABAB_EDGES: u32 = 1800;
/// 残留ドリフトの恒久修正 (20260704): 捕捉プログラムを一周コスト対称版にする。X の 0 跨ぎ (68.7s 毎) の
/// +1 cycle が low 待ちループだけに乗る非対称 × ピンの duty 差で、別ピンを見る 2 カウンタが
/// 0.87 回/分 × Δduty × 8ns (実測 5.6ns/min) で開くのを、両ループ +1 cycle に揃えて duty 非依存にする。
/// 全カウンタが一律 8ns/68.7s (≈0.12ppb) 遅れるが、共通分は読み差から消え周波数推定が吸収する。
/// もう 1 つの対処はパルス幅合わせ (PPS_PULSE_NS を GPS-R と同じ low 100ms = high 900ms にする) で、
/// どちらも実測で march ≈ 0 を確認済み。幅合わせは受信機の波形に依存するため、既定はこちらの対称版。
const WRAP_BALANCED_CAPTURE: bool = true;

/// 最新の GPS PPS エッジの生カウンタ値 (SM0)。stage② の PIO 位相計測で gen_capture が参照する。
static C0_GPS: AtomicU32 = AtomicU32::new(0);
/// GPS PPS エッジの世代カウンタ。出力エッジ間に進んでいなければ GPS 欠落 = C0_GPS は古い → 補正に使わない。
static C0_GEN: AtomicU32 = AtomicU32::new(0);

/// 実験 (KEXP): GP2 を常時観測する第3 capture SM (SM3) が捕えた最新の GPS PPS 生カウンタ。
/// **純観測** — K/hwphase/servo/recal/set_next_period のどの式にも入らない。pps_task が毎エッジ
/// drain-to-latest し、Locked エッジで C0_GPS と同じ世代境界 (C0_GEN の Release) の内側で store する。
/// 読者は gen_capture_task の KEXP ログ 1 箇所のみ (grep で load 1・store 1 の計 2 箇所を機械確認できる)。
/// torn-read 不在は「pps_task と gen_capture_task が同一 executor (spawner_high)」に依存 — 将来どちらかを
/// 別 executor へ移すと c0/c3 の一貫性が壊れる。
static C3_GPS: AtomicU32 = AtomicU32::new(0);
/// そのエッジで SM3 FIFO から drain した個数 (1=正常, 0=SM3 未着/取り損ね→C3_GPS は前値のまま,
/// >1=pps_task が 1s 以上 stall した backlog で latest が先行エッジになりミスペア疑い)。解析は c3n≠1 を棄却。
static C3_N: AtomicU32 = AtomicU32::new(0);

/// 受信適応帯域: 1=悪受信 (遅い/減衰ループ)、0=良受信 (速いループ)。pps_task が間隔ジッタから set、
/// gen_capture_task が読んで (i_den,d_den) を切替える。
static RECEPTION_BAD: AtomicU32 = AtomicU32::new(0);
/// 直近の GPS 間隔ジッタ (ns, ログ用)。
static RECEPTION_JITTER: AtomicU32 = AtomicU32::new(0);

/// RP2040 内蔵温度センサの ADC 値を **N 回積算平均した fractional 値** (12bit ADC code を ×TEMP_RAW_SCALE
/// した固定小数)。temp_task が **DMA free-running の連続背景タスク**として常時更新し (1 block ≈ TEMP_DMA_N
/// サンプル)、gen_capture_task が温度FF入力として規律コアへ渡し、PPSGEN ログにも出す。**生の 12bit code
/// に戻すには TEMP_RAW / TEMP_RAW_SCALE** (=×256、生 code に戻すには /256)。重オーバーサンプル
/// (N=TEMP_DMA_N) で ADC ノイズ (~1-2 LSB) を natural dither にして √N で sub-LSB 分解能を得る
/// (1/256 LSB ≈ 0.002°C)。℃ 換算は raw code を `27-(code*3.3/4096-0.706)/0.001721`。水晶/受信機でなく
/// MCU 温度で単発はノイジー。**規律コア (gnssdo) は温度非依存** — 温度の取り込みは firmware 配線で、
/// 温度FF も runtime トグル (TEMP_FF_ENABLE) で明示的に有効化したときだけ効く。
static TEMP_RAW: AtomicU32 = AtomicU32::new(0);
/// TEMP_RAW の固定小数スケール (平均 ADC code をこの倍率で格納)。解析・℃換算は割って戻す。
/// 256 = 8 fractional bits ≈ 1/256 LSB ≈ 0.002°C (重オーバーサンプルの sub-LSB 分解能を保持)。
const TEMP_RAW_SCALE: u32 = 256;
/// temp_task が 1 回の DMA 転送で取る ADC サンプル数 (重オーバーサンプリングの平均窓)。
/// 16384 で √16384=128 → 量子化ノイズの平均化が ×256 スケールの分解能を裏打ちする。
/// buf は u16 × 16384 = 32KB (RP2040 RAM 264KB に収まる)。StaticCell で .bss に確保。
const TEMP_DMA_N: usize = 16384;

/// 内蔵温度センサを **DMA free-running の最大レート連続サンプリング**で TEMP_RAW に積む背景タスク。
/// ADC を最速 (div=0 → ~500kSa/s 連続変換) で回し、**DMA がハードウェアで buf に書く (CPU 負荷ゼロ)**。
/// task は `adc.read_many(.., div=0, dma).await` で **DMA 完了を await するだけ**で、転送中 (16384 サンプル
/// ≈ 33ms) は thread executor が PPS 捕捉・NMEA・制御など他タスクを自由に回せる → **starvation 無し**。
/// 完了したら 16384 サンプルを block-average し TEMP_RAW_SCALE(=256) 倍の fractional 値で格納、即次 DMA
/// (Timer スリープ不要 = DMA await が手放しになるため)。CPU が触るのは setup と平均ループ (~16384 加算、
/// 数十µs) だけで ADC duty ~100% でも CPU duty ~0%。旧実装は CPU が 1 サンプルずつ adc.read().await を
/// N 回回して thread executor を専有し、レートを上げると制御/NMEA を遅らせロックを落としていた
/// (busy-loop 版で実機 lock 23%)。本実装はその CPU 律速を DMA で根治する。
#[embassy_executor::task]
async fn temp_task(
    mut adc: embassy_rp::adc::Adc<'static, embassy_rp::adc::Async>,
    mut ch: embassy_rp::adc::Channel<'static>,
    mut dma: embassy_rp::dma::Channel<'static>,
) {
    // DMA 転送先。32KB を task の arena スタックに積まず StaticCell で .bss に置く。
    static BUF: StaticCell<[u16; TEMP_DMA_N]> = StaticCell::new();
    let buf = BUF.init([0u16; TEMP_DMA_N]);
    loop {
        // div=0 → ADC 最速連続 (div<96 は 0 と同義)。転送完了まで executor を手放す (CPU 負荷ゼロ)。
        if adc
            .read_many(&mut ch, &mut buf[..], 0, &mut dma)
            .await
            .is_ok()
        {
            // 全 N サンプルを積算。1 サンプル 12bit ≤ 4095, N=16384 → sum ≤ 4095*16384 ≈ 67.1M < u32::MAX。
            let mut sum: u32 = 0;
            for &s in buf.iter() {
                sum += (s & 0x0fff) as u32; // ADC FIFO の有効 12bit のみ採る
            }
            // 平均 × SCALE = sum*256/N。sum*256 ≤ 67.1M*256 ≈ 17.2e9 が u32 を超えるので u64 で計算。
            // 結果 ≤ 4095*256 = 1,048,320 は u32 に収まる。生 code に戻すには /256。
            let scaled = (sum as u64 * TEMP_RAW_SCALE as u64 / TEMP_DMA_N as u64) as u32;
            TEMP_RAW.store(scaled, Ordering::Relaxed);
        }
        // スリープ無し: 次の DMA の await が即座に executor を手放すので、ここで Timer を挟む必要はない。
    }
}

/// 非PIO 素朴 1PPS 出力タスク (S0/S1)。`Timer::after` で GP3 をトグルする。`apply_freq` なら
/// `NaivePeriodDither` で水晶推定 (`predicted_freq_mppb`) を embassy-tick 周期へ一次 sigma-delta で
/// 適用 (S1)、さもなくば公称 1Hz で自走 (S0)。立ち上がり時刻の秒内位相を `OUT_SUBSEC_NS` に publish。
/// 出力エッジの瞬時 jitter は M0+ の soft scheduling (~µs) 律速 = この経路が見せたい計測下限
/// (dither は **平均**周波数を sub-tick へ細かくするが、瞬時 jitter は別物)。
#[embassy_executor::task]
async fn naive_pps_out_task(mut out: Output<'static>) {
    let tick_hz = Duration::from_secs(1).as_ticks() as u32;
    let pulse_ticks = Duration::from_millis(NAIVE_PPS_PULSE_MS).as_ticks();
    let mut dither = NaivePeriodDither::new();
    // 立ち上がりを **絶対 deadline** で打つ。相対 Timer::after だと毎周期に executor の wake/lock
    // overhead (~1 tick) が平均周期へ累積し、dither の sub-tick 解像を系統バイアスで汚す。絶対 deadline
    // なら overhead は各エッジの jitter に留まり (見せたい soft 下限そのもの)、平均周期 = dither になる。
    let mut next_rise = Instant::now();
    loop {
        Timer::at(next_rise).await;
        out.set_high();
        let edge_ns = now_local_ns();
        OUT_SUBSEC_NS.store((edge_ns % 1_000_000_000) as u32, Ordering::Relaxed);
        OUT_GEN.fetch_add(1, Ordering::Release);
        // 次の立ち上がり時刻 = 今回の deadline + dither 周期 (S1 は水晶推定を適用、S0 は公称)。
        let freq_mppb = if apply_freq(PRECISION_STAGE) {
            // 操舵には clamp 版を使う (fast 熱過渡で predicted が過反応しても出力周期を殴らない)。
            // raw predicted は holdover/診断のために steering_freq_mppb 内で温存される。
            CLOCK.lock(|g| g.borrow().steering_freq_mppb())
        } else {
            0
        };
        let period_ticks = dither.next_ticks(tick_hz, freq_mppb) as u64;
        // dither 実働の直接証跡: 実際に次エッジへ適用した周期を publish (PPSGEN の dith_ticks)。
        NAIVE_DITH_TICKS.store(period_ticks as u32, Ordering::Relaxed);
        next_rise += Duration::from_ticks(period_ticks);
        // パルス幅 (立ち下がりの時刻精度は不問: 規律対象は立ち上がりエッジ)。
        Timer::after(Duration::from_ticks(pulse_ticks)).await;
        out.set_low();
    }
}

/// 非PIO 素朴 1PPS 入力タスク (S0/S1)。GP2 の GPS 1PPS を GPIO 割込 + Instant で soft 捕捉
/// (µs オーダ下限 = M0+ critical-section 全 IRQ マスク。σ は負荷で ~2〜10µs)。連続エッジ差から interval を作り、production と
/// **同一の** `on_pps_edge` (raw 非参照を host テストで確認済) へ縮退 TimedEdge を渡して水晶 ppb を
/// 規律コアに食わせる (S0=推定のみ・周期不適用 / S1=周期へ適用)。出力 vs GPS の開ループ modulo phase を
/// `fold_phase_ns` で作り hwphase 欄に soft err として載せ、段共通スキーマの PPS/PPSGEN 行で出す。
#[embassy_executor::task]
async fn naive_pps_in_task(mut gps: Input<'static>) {
    let mut count: u32 = 0;
    let mut last_edge_ns: Option<u64> = None;
    loop {
        gps.wait_for_rising_edge().await;
        let edge_ns = now_local_ns();
        let interval_ns = match last_edge_ns {
            Some(prev) => edge_ns.saturating_sub(prev),
            None => 0,
        };
        last_edge_ns = Some(edge_ns);
        // production と同一の規律経路。capture==query の縮退 (両者 Instant)、raw=0 (非参照)。
        let step = CLOCK.lock(|g| {
            g.borrow_mut().on_pps_edge(
                TimedEdge {
                    raw: 0,
                    interval_ns,
                    edge_ns,
                },
                edge_ns,
            )
        });
        count += 1;
        // soft 開ループ modulo phase (出力 vs この GPS エッジ)。出力が 1 度も出ていなければ載せない。
        // fold は秒内なので出力/入力の ±1 秒帰属誤りに不変 (PLAN 採用#5、同秒ペアリング不要)。
        let out_gen = OUT_GEN.load(Ordering::Acquire);
        let soft_err = if out_gen > 0 {
            fold_phase_ns(
                OUT_SUBSEC_NS.load(Ordering::Acquire) as i64,
                (edge_ns % 1_000_000_000) as i64,
            )
        } else {
            0
        };
        let (state, missed): (&str, u32) = match step.event {
            PpsEvent::First => ("First", 0),
            PpsEvent::Locked { .. } => ("Locked", 0),
            PpsEvent::Irregular { missed, .. } => ("Irregular", missed),
            PpsEvent::NonMonotonic { .. } => ("NonMono", 0),
        };
        let freq = step.freq.map_or("none", |fu| fu.as_str());
        info!(
            "PPS count={} interval_us={} interval_ns={} state={=str} missed={} freq={}",
            count,
            interval_ns / 1000,
            interval_ns,
            state,
            missed,
            freq
        );
        // 最初のエッジは interval が無い (規律も phase も意味がない) のでスキップ。
        if interval_ns == 0 {
            continue;
        }
        // 段共通スキーマの PPSGEN 行 (PIO 経路と同一フィールド列)。非PIO で意味のない制御項は 0。
        // hwphase 欄 = soft open-loop modulo phase、olmod=1 (常に開ループ modulo phase)。
        let (slope_mppb, temp_k, ff_delta_mppb, steer_ff_mppb) = CLOCK.lock(|g| {
            let g = g.borrow();
            (
                g.freq_slope_mppb(),
                g.temp_k_mppb_per_unit(),
                // ff_delta = raw predicted の操舵寄与 (clamp 前、過反応を診断ログに残す)。
                g.predicted_freq_mppb() - g.freq_mppb(),
                // steer_ff = 実際に操舵へ渡る clamp 後の FF 偏差 (clamp 発火の A/B 証跡)。
                g.steering_freq_mppb() - g.freq_mppb(),
            )
        });
        info!(
            "PPSGEN count={} interval_ns={} dev_ns={} phase_ns={} hwphase_ns={} trim_ppb={} cidx={} p_ns={} d_ns={} trim_mppb={} slope_mppb={} raw_lag={} lk={} state={} jit={} rxbad={} inj_ns={} kick_ns={} temp_raw={} dynmode={} temp_k={} ff_delta={} olmod={} stage={} recal_eff={} dith_ticks={} steer_ff={}",
            count,
            interval_ns,
            interval_ns as i64 - 1_000_000_000,
            0i64,     // phase_ns: 非PIO に Instant ベースの出力位相測定はない
            soft_err, // hwphase_ns: soft 開ループ modulo phase
            0i64,     // trim_ppb
            0u32,     // cidx
            0i64,     // p_ns
            0i64,     // d_ns
            0i64,     // trim_mppb
            slope_mppb,
            0i32, // raw_lag
            0u8,  // lk (位相ロックなし)
            0u8,  // state
            0u32, // jit
            0u32, // rxbad
            0i64, // inj_ns
            0i64, // kick_ns
            TEMP_RAW.load(Ordering::Relaxed),
            mt3333::DYN_MODE.load(Ordering::Relaxed),
            temp_k,
            ff_delta_mppb,
            1u8, // olmod: naive は常に開ループ modulo phase
            PRECISION_STAGE as u32,
            recal_eff(PRECISION_STAGE),
            NAIVE_DITH_TICKS.load(Ordering::Relaxed), // dith_ticks: 実適用 dither 周期 (sigma-delta 証跡)
            steer_ff_mppb // steer_ff: clamp 後の操舵 FF 偏差 (steering − level)
        );
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

/// 走行中 capture SM の jmp_pin だけを execctrl 経由で切替える (GP2↔GP4)。**set_config は使わない**:
/// use_program 済み config だと末尾で exec_jmp(origin) を呼び、走行中 SM の PC を先頭へ強制ジャンプさせる。
/// capture SM で pin high 中にこれをやると先頭 jmp pin rising が即 capture 経路へ入り、4 サイクル無減算 =
/// 自走カウンタ X が −2 tick 損し、偽 push も 1 個出る (KPOKE 実験 20260704 で実証)。jmp_pin レジスタだけ
/// 書けば program/clkdiv/pindirs は不変 (cfg_gp2/cfg_gp4 は jmp_pin 以外同一) なので X を乱さず切替えられる。
fn switch_jmp_pin(sm: usize, pin: u8) {
    embassy_rp::pac::PIO0
        .sm(sm)
        .execctrl()
        .modify(|w| w.set_jmp_pin(pin));
}

/// PIO がラッチした PPS エッジを読み、間隔を ns で求めて出す + 周波数を規律するタスク。
#[embassy_executor::task]
async fn pps_task(
    mut capture: TimedPpsCapture<'static, PIO0, 0>,
    mut sm3: StateMachine<'static, PIO0, 3>,
) {
    let mut count: u32 = 0; // ログ用エッジ連番
    let mut iv_buf = [0i64; RX_JIT_WIN]; // 直近 Locked エッジの interval_ns (受信品質ジッタ用 ring buffer)
    let mut iv_cnt: usize = 0;

    loop {
        // wait_edge + timeline.observe は TimedPpsCapture に委譲。生カウンタは edge.raw で取れる。
        let edge = capture.next_edge().await;
        let edge_raw = edge.raw;
        // エポックのアンカー用に Instant を読む (µs ジッタは絶対オフセットのみに効く)。
        let query_ns = now_local_ns();

        // 実験(KEXP): 同じ GP2 を見る SM3 を毎エッジ最新まで drain し、SM0 と lockstep に保つ。
        // Locked 以外のエッジでも drain して残留を残さない(→定常で c3n==1)。store は下の Locked ゲート内のみ。
        // try_pull のみ (wait_pull は FIFO 割込を有効化し新しい起床経路を作るので絶対に使わない)。
        let mut c3: Option<u32> = None;
        let mut c3n: u32 = 0;
        while let Some(v) = sm3.rx().try_pull() {
            c3 = Some(v);
            c3n += 1;
        }

        // 周波数規律 (Locked のみ・復帰 quarantine) + 次の RMC 用にエッジ記録を PpsGpsdo に委譲。
        // PPS_TS signal は不要に: エッジは共有 state に記録され、main の feed_nmea が拾う。ここは log だけ。
        count += 1;
        let step = CLOCK.lock(|g| g.borrow_mut().on_pps_edge(edge, query_ns));
        // stage② 共有 C0 は **Locked エッジのみ** publish。spurious/Irregular な GP2 捕捉を loopback の
        // 基準にすると、ループがそれと対にして出力が誤った点へ寄り、しかも capture SM の余分捕捉で SM0/SM2
        // の実効 K が runtime ドリフトする (オシロで offset② が ~µs まで育ち boot でリセット、を実機確認)。
        if matches!(step.event, PpsEvent::Locked { .. }) {
            // 実験(KEXP): C3 store は C0_GEN の Release より前に置く (Release が C3_GPS/C3_N/C0_GPS を一括 publish)。
            if let Some(v) = c3 {
                C3_GPS.store(v, Ordering::Relaxed); // c3n==0 のとき前値を保持 (下の C3_N が stale を示す)
            }
            C3_N.store(c3n, Ordering::Relaxed);
            C0_GPS.store(edge_raw, Ordering::Relaxed);
            C0_GEN.fetch_add(1, Ordering::Release); // gen++ (Release: gen 進行で直前の C3/C0 store が見える)
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
                let now_bad = if was_bad {
                    jit > RX_JIT_LO_NS
                } else {
                    jit > RX_JIT_HI_NS
                };
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
    // SM2 を GP2 へ (jmp_pin だけ切替。set_config は exec_jmp(origin) で走行中 SM の X に −2 tick の段を作る
    // ため使わない。KPOKE 実験 20260704 で実証)。cfg_gp2 と cfg_gp4 は jmp_pin 以外同一なので register 直書きで足りる。
    switch_jmp_pin(2, cfg_gp2.get_exec().jmp_pin);
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
    // SM2 を GP4 へ戻す (jmp_pin だけ切替、X 保持)。drain して次の GP4 捕捉が新鮮な出力エッジになるように。
    switch_jmp_pin(2, cfg_gp4.get_exec().jmp_pin);
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
        warn!(
            "PHASE_K recal reject: spread={} n={}",
            spread,
            samples.len()
        );
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
    let k0_boot: u32 = k; // shadow recal 用: 起動時 K0 を保持し K_shadow の drift を dk=K_shadow-K0 で出す。
    let mut edges_since_shadow: u32 = 0;
    // SM3 shadow の状態: cadence カウンタと、GP2 滞在の残エッジ数 (0 = GP4 常駐中)。
    let mut edges_since_sh3: u32 = 0;
    let mut sh3_window_left: u32 = 0;
    // 温度 FF A/B の状態。初期値は段ゲートの設定 (stage-3 なら off 始まり)。
    let mut edges_since_tffab: u32 = 0;
    let mut tffab_on: bool = temp_ff(PRECISION_STAGE);
    // 直近の出力周期語。再較正中の holdover 生 push に再利用する。必ず set_next_period (下) で代入されてから
    // recal_k に渡る (再較正はループ末尾＝周期語確定後にしか発火しない) ので、初期値は不要。
    let mut last_period: u32;
    let mut edges_since_recal: u32 = 0;
    // 間欠ループバック実験の状態。exp_open=開窓(周波数FFのみ)、edges_in_mode=現窓の経過 locked edges、
    // toff_idx=EXP_TOFF_SCHEDULE 内の位置 (開窓ごとに巡回)。
    let mut exp_open = false;
    let mut exp_edges_in_mode: u32 = 0;
    let mut exp_toff_idx: usize = 0;
    let mut exp_held_trim: i64 = 0; // 開窓で保持する周波数補正 (閉窓トリムの平均 = steady integrator)
    let mut exp_trim_sum: i64 = 0; // 閉窓中のトリム積算 (平均用)
    let mut exp_trim_n: u32 = 0;
    // 出力位相の制御器 (gnssdo::Controller)。sub-cycle period 生成 (dither) + period word push は
    // SteeredPpsOutput (rp-pps) が内包するので、ここは freq_mppb と phase_corr を渡すだけ。本番は
    // slot 0 (現行 firmware の PID+Smith チューニング)。CONTROLLER_SWEEP=true で locked 中に巡回する。
    let mut sweep_idx: usize = 0; // SWEEP_SCHEDULE 内の位置。実 cidx = SWEEP_SCHEDULE[sweep_idx]。
    // 段ゲート: S3+ は production の PLL (slot0)、S2 は開ループFF (周波数FFのみ・位相補正なし)。
    // S5 では phase_servo(5)=true → 現行と bit 一致 (const assert で保証)。
    let mut controller = if phase_servo(PRECISION_STAGE) {
        make_controller(SWEEP_SCHEDULE[sweep_idx])
    } else {
        Controller::OpenLoop(OpenLoopFf::new())
    };
    // boot で初期セグメントの effective config を 1 行出す (固定運用=CONTROLLER_SWEEP=false でも解析が
    // どの制御器 config で走ったか分かるように)。以後の切替は下の CTRLSWEEP 行で出す。
    {
        let cidx = SWEEP_SCHEDULE[sweep_idx];
        let (i_den, d_den, kp_inv, smith) = CTRL_GRID[cidx % CONTROLLER_LIST_LEN];
        info!(
            "CTRLSWEEP idx={} name={} forced={} i_den={} d_den={} kp_inv={} smith_edges={}",
            cidx as u32,
            controller.name(),
            0u8,
            i_den,
            d_den,
            kp_inv,
            smith
        );
    }
    let mut edges_since_sweep: u32 = 0; // セグメント内の **locked** エッジ数 (通常の切替契機)
    let mut edges_seg_total: u32 = 0; // セグメント内の **総** エッジ数 (非ロック config の force-advance 用)
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
            warn!(
                "PPSGEN drained {} stale output edge(s) (loop lagged)",
                dropped
            );
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
            let diff =
                (k_target.wrapping_sub(k) as i32).clamp(-RECAL_K_SLEW_TICKS, RECAL_K_SLEW_TICKS);
            k = k.wrapping_add(diff as u32);
        }
        // stage② PIO ハード位相: 出力エッジ と 最新 GPS エッジ の生カウンタ差 (mod 1秒) を ns に。
        // Instant を介さないので executor のウェイクアップ遅延 (~ms) に汚されず 16ns 分解能。
        let c0 = C0_GPS.load(Ordering::Relaxed);
        // 実験(KEXP): 同じ await 無し snapshot 区間 (cur_gen Acquire 〜 PPSGEN emit) で C3 も読む。
        // (cur_gen, c0, c3, c3n) は torn read なしの一貫 snapshot。純観測で制御式には入らない。
        let c3 = C3_GPS.load(Ordering::Relaxed);
        let c3n = C3_N.load(Ordering::Relaxed);
        // hwphase は loopback (出力 vs GPS) の相対位相。静的 pad/経路スキューは scope チューニングせず
        // 受け入れる (旧 ABS_OFFSET は除去、recal が実効 K のドリフトを bound)。
        let k_eff = k;
        let hwphase_ns = rp_pps::loopback_phase_ns(c0, x, k_eff, clk);
        // ペアリング診断: fold 前の生 tick 差。正しい隣接エッジ対なら小さい(=真の位相)、ミスペア(欠落/
        // drain/再ロック後に非隣接エッジと対)だと ≈±1秒。fold がそれを隠し ppm×1s 残差にループがロック
        // する (9.5µs バグ族)。|raw_lag| が ~10ms 超なら gross ミスペアとして invalid 化し、loop に渡さない。
        // hwphase=0 ⟺ GP3=GPS を loopback 自身で(scope に頼らず)保つ。raw_lag はログして機構を観測。
        let raw_lag = rp_pps::loopback_raw_lag_ticks(c0, x, k);
        let max_lag = (clk / 200) as i32; // capture tick ≈ clk/2 なので clk/200 ≈ 10ms
        let paired = raw_lag.unsigned_abs() <= max_lag as u32;
        // 段ゲート (採用#2): S3+ (位相サーボ) のみミスペアを棄却する。S2 (開ループFF) は paired=false でも
        // hwphase をログし続ける。olmod=1 は「この hwphase 行が開ループ modulo phase」(=サーボ不在で
        // 拘束されていない) ことを表し、S2 の**全行**で立つ (paired か否かは別途 raw_lag が示す)。S5 では
        // phase_servo=true ゆえ olmod は常に 0。
        let drop_on_mispair = phase_servo(PRECISION_STAGE);
        let olmod = !phase_servo(PRECISION_STAGE);
        // 旧手法 (比較用 emit のみ): 出力エッジの UTC 時刻 (Instant 経由) → 秒境界からのズレ。
        let (pred_mppb, steer_mppb, level_mppb, temp_k, slope_mppb, phase) = CLOCK.lock(|g| {
            let g = g.borrow();
            (
                g.predicted_freq_mppb(),  // raw (holdover/診断用、clamp なし)
                g.steering_freq_mppb(),   // clamp 後 (出力周期の操舵に使う)
                g.freq_mppb(),            // α-β level (温度FF lead を含まない)
                g.temp_k_mppb_per_unit(), // 学習した温度係数 (0=未学習)
                g.freq_slope_mppb(),
                g.now_from_query_ns(now_local_ns()).map(snap_to_second_ns),
            )
        });
        // 温度FF の操舵寄与 (predicted − level)。temp_k と併せて温度FF が実際に効いているかの診断。
        // ff_delta は raw predicted の寄与 (clamp 前、過反応を残す)、steer_ff は実操舵の clamp 後寄与。
        let ff_delta_mppb = pred_mppb - level_mppb;
        let steer_ff_mppb = steer_mppb - level_mppb;
        // 制御に使う位相: PIO ハード(stage②)。PHASE_USE_HW=false で旧 Instant 測定に切替 (比較計測用)。
        let ctrl = if PHASE_USE_HW {
            hwphase_ns
        } else {
            phase.unwrap_or(0)
        };
        // valid = この位相サンプルが信用できるか (GPS 新鮮 + 間隔健全 + 隣接エッジ対)。!valid は holdover
        // ホールド。paired=false (gross ミスペア) は fold が隠す ppm×1s 残差を避けるため捨てる。
        if drop_on_mispair && !paired {
            warn!("PPSGEN mispair raw_lag={} ticks (>~10ms) -> drop", raw_lag);
        }
        // 間欠実験の開窓中は制御器を hold (valid=false) して windup を避け、出力は周波数FFのみにする。
        // hwphase は上で計算済みなので、開ループのドリフトはログ/scope に出る (適用しないだけ)。
        let exp_hold = INTERMITTENT_EXP && exp_open;
        // S3+ は paired を valid 条件に含める (ミスペア棄却)。S2 は含めない (開ループFF は err を使わないので
        // 無害、hwphase は modulo phase 列として残す)。S5 では drop_on_mispair=true で現行と同一。
        let valid = fresh && sane && c0 != 0 && (!drop_on_mispair || paired) && !exp_hold;
        // 選択中の制御器に位相誤差を渡す (gnssdo::Controller。本番=slot0、CONTROLLER_SWEEP 中は巡回)。
        let u = controller.step(ControlInput {
            err_ns: ctrl,
            valid,
        });
        // 温度を規律コアへ連続供給 (自己クロック=連続量という前提)。ロック中の各エッジで最新 fractional
        // TEMP_RAW を update_temp に渡す (~1Hz、温度FF の EMA は呼び出し回数単位なので PPS 1Hz と整合)。
        // TEMP_FF_ENABLE=false (実験A) なら model は値の記録/learn のみで predicted_freq は不変。B/C で
        // enable すると温度の leading 推定が holdover/出力周期に効く。
        if u.locked {
            let t = TEMP_RAW.load(Ordering::Relaxed) as i64;
            CLOCK.lock(|g| g.borrow_mut().update_temp(t));
        }
        // 周波数 = 水晶推定の 1 サンプル先予測 (mppb, ppb 丸めなし) + 制御器の周波数トリム (milli-ppb)。
        // 温度ランプ中、単一 EMA だと推定が遅れて出力位相が ramp 速度に比例してずれる (offset② のドリフト)。
        // DisciplinedClock の α-β が傾きを先回りするので、出力周期にその予測周波数を入れて定常誤差を抑える。
        // 位相補正は dither が cycle に変換して引く。
        // 開窓 (exp_hold): 位相補正なし。周波数は EXP_HOLD_TRIM=true なら学習済みトリムを保持 (pred + 保持 trim)、
        // false なら pred のみ (補正を丸ごと捨てる)。閉窓/本番は通常どおり制御器のトリム + 位相。
        // 操舵には clamp 版 (steer_mppb) を使う。fast 熱過渡で raw predicted が過反応しても出力周期を
        // 殴らない。raw predicted は holdover/診断 (ff_delta) のために pred_mppb として温存している。
        let freq_mppb = if exp_hold {
            steer_mppb + if EXP_HOLD_TRIM { exp_held_trim } else { 0 }
        } else {
            steer_mppb + u.trim_mppb
        };
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
        let pcorr_ns = if exp_hold { 0 } else { u.pcorr_ns };
        // 残ドリフト調査: 出力周波数へループに隠して既知バイアスを注入 (freq_mppb には足すが last_applied/loop 状態には非反映)。
        let inj_mppb = if INJECT_FREQ_MPPB != 0 && u.locked && count >= INJECT_START_COUNT {
            INJECT_FREQ_MPPB
        } else {
            0
        };
        last_period = output.set_next_period(freq_mppb + inj_mppb, pcorr_ns + prbs_inj + kick_inj);
        if INJECT_FREQ_MPPB != 0 {
            info!("KINJ count={} inj_mppb={}", count, inj_mppb);
        }
        if let Some(iv) = interval_ns {
            count += 1;
            // 制御信号を全部出力 → ホストでプラント同定 + 制御器比較 + 項別比較ができる。
            // 互換のため既存 5 項 count/interval/dev/phase/hwphase を先頭に残す。cidx=制御器インデックス
            // (CONTROLLER_SWEEP の segment 鍵)、state=制御器の内部状態コード (ab_boost の boost 等)。
            // **末尾 append-only** (PLAN 採用#10): prefix と既存項は byte 不変。olmod=S2 開ループ modulo
            // phase 列フラグ、stage/recal_eff = この段の実効設定 (ログが一次情報・採用#3/#4)。
            info!(
                "PPSGEN count={} interval_ns={} dev_ns={} phase_ns={} hwphase_ns={} trim_ppb={} cidx={} p_ns={} d_ns={} trim_mppb={} slope_mppb={} raw_lag={} lk={} state={} jit={} rxbad={} inj_ns={} kick_ns={} temp_raw={} dynmode={} temp_k={} ff_delta={} olmod={} stage={} recal_eff={} dith_ticks={} steer_ff={}",
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
                kick_inj,
                TEMP_RAW.load(Ordering::Relaxed),
                mt3333::DYN_MODE.load(Ordering::Relaxed),
                temp_k,
                ff_delta_mppb,
                olmod as u8,
                PRECISION_STAGE as u32,
                recal_eff(PRECISION_STAGE),
                0u32, // dith_ticks: PIO 経路は dither 不使用 (両経路スキーマ一致のため 0)
                steer_ff_mppb  // steer_ff: clamp 後の操舵 FF 偏差 (steering − level)
            );
            // 実験(KEXP): 実効 K ドリフト切り分け用の生カウンタ行。PPSGEN と同 count で 1:1 join できる。
            // 純観測 (c3/c3n は制御に非関与)。PPSGEN のフォーマット/引数は 1 byte も変えていない (別 info! を後置)。
            info!(
                "KEXP count={} gen={} c0={} c2={} c3={} c3n={} k={} kt={} fresh={} paired={} lk={}",
                count,
                cur_gen,
                c0,
                x,
                c3,
                c3n,
                k,
                k_target,
                fresh as u8,
                paired as u8,
                u.locked as u8
            );
        }
        last_x = Some(x);
        // 定期 K 再較正: locked 中、RECAL_EDGES 出力エッジ毎に SM2 を GPS へ戻して実効 K を測り直す。
        // 実効 K は温度で這う (#5 後も ~5ns/min) ので boot 一発較正では追えない。再較正中は holdover。
        // 実験: 制御器巡回。**全エッジ評価**で切替: (a) locked エッジが CONTROLLER_SWEEP_EDGES に達したら通常
        // 切替、(b) 総エッジが 3× に達したら **force-advance** — 非ロックの config (発散する対照など) がスイープを
        // stall させないため。一定受信下で各手法の hwphase / PRBS h[k] を比較する。切替は start_segment: 出力
        // 周波数が連続になるよう residual trim = 直近適用周波数 − 新手法の現 pred を継ぎ、観測器/Smith/lock を blank。
        if CONTROLLER_SWEEP {
            edges_seg_total += 1;
            if u.locked {
                edges_since_sweep += 1;
            }
            if edges_since_sweep >= CONTROLLER_SWEEP_EDGES
                || edges_seg_total >= CONTROLLER_SWEEP_EDGES * 3
            {
                let forced = edges_since_sweep < CONTROLLER_SWEEP_EDGES;
                edges_since_sweep = 0;
                edges_seg_total = 0;
                edges_since_kick = 0; // 新セグメント先頭で kick 位相を揃える
                sweep_idx = (sweep_idx + 1) % SWEEP_SCHEDULE.len();
                let cidx = SWEEP_SCHEDULE[sweep_idx];
                controller = make_controller(cidx);
                controller.start_segment(gnssdo::ControlInit {
                    // 出力基準は clamp 版 steer_mppb なので継ぎ目の連続性も steer 基準で取る。
                    // pred_mppb 基準だと clamp 発火中に (pred−steer) の差分が次セグメント初期 trim へ
                    // ステップ混入する (sweep 時のみ顕在)。
                    residual_trim_mppb: last_applied_freq_mppb - steer_mppb,
                });
                let (i_den, d_den, kp_inv, smith) = CTRL_GRID[cidx % CONTROLLER_LIST_LEN];
                info!(
                    "CTRLSWEEP idx={} name={} forced={} i_den={} d_den={} kp_inv={} smith_edges={}",
                    cidx as u32,
                    controller.name(),
                    forced as u8,
                    i_den,
                    d_den,
                    kp_inv,
                    smith
                );
            }
        }
        // shadow recal (残ドリフト調査): 適用中の recal が無い段 (S3) で、K を測るだけ測って適用しない。
        // k_target は据え置くので loop は起動時 K0 のまま = pin は observed の +5-6ns/min で drift し続ける。
        // recal_k は測定値 (nk) か却下時 k_target を返す。dk=k_meas-K0 を KSHADOW でログ、gap との照合は host で。
        if SHADOW_RECAL && u.locked && !recal_on(PRECISION_STAGE) {
            edges_since_shadow += 1;
            if edges_since_shadow >= SHADOW_EDGES {
                edges_since_shadow = 0;
                let k_meas = recal_k(
                    &mut sm,
                    &mut output,
                    &cfg_gp2,
                    &cfg_gp4,
                    k_target,
                    last_period,
                )
                .await;
                info!(
                    "KSHADOW count={} k_meas={} k0={} dk={}",
                    count,
                    k_meas,
                    k0_boot,
                    k_meas.wrapping_sub(k0_boot) as i32
                );
                last_x = None; // GP4 捕捉が測定 gap を跨ぐので次 interval を無効化 (real recal と同じ)
                last_gen = C0_GEN.load(Ordering::Acquire);
            }
        }
        // SM3 shadow (残ドリフト調査): SM3 を cadence 毎に GP2 へ WINDOW エッジだけ向けて K03=c0−c3 を測る。
        // SM2/servo/last_x は一切触らない (SM3 は制御非関与)。データは KEXP 行に出るので、ここは切替と目印だけ。
        // 切替はどちら向きも出力エッジ処理直後 = 両パルスの high 窓内 (100ms) なので、SM3 は high ループの
        // まま継ぎ目なく移り、余分な捕捉パス通過は起きない。
        if SHADOW3 && u.locked {
            if sh3_window_left > 0 {
                sh3_window_left -= 1;
                if sh3_window_left == 0 {
                    switch_jmp_pin(3, cfg_gp4.get_exec().jmp_pin);
                    info!("KSH3 count={} on=4", count);
                }
            } else {
                edges_since_sh3 += 1;
                let cadence = if count < SHADOW3_PHASE_B_COUNT {
                    SHADOW3_EDGES_A
                } else {
                    SHADOW3_EDGES_B
                };
                if edges_since_sh3 >= cadence {
                    edges_since_sh3 = 0;
                    sh3_window_left = SHADOW3_WINDOW_EDGES;
                    switch_jmp_pin(3, cfg_gp2.get_exec().jmp_pin);
                    info!("KSH3 count={} on=2", count);
                }
            }
        }
        // 温度 FF の長時間 A/B: locked エッジを数え、区間ごとに適用だけを反転する。
        if TEMPFF_ABAB && u.locked {
            edges_since_tffab += 1;
            if edges_since_tffab >= TEMPFF_ABAB_EDGES {
                edges_since_tffab = 0;
                tffab_on = !tffab_on;
                CLOCK.lock(|g| g.borrow_mut().set_temp_ff_enable(tffab_on));
                info!("TFFAB count={} temp_ff={}", count, tffab_on as u8);
            }
        }
        if INTERMITTENT_EXP {
            // 間欠ループバック実験: 閉窓 (EXP_TON_EDGES) ↔ 開窓 (EXP_TOFF_SCHEDULE[idx]) を巡回。
            // 開窓中は recal を止め周波数FFのみ (上の exp_hold)。窓入口 (開→閉) で K を測り直す。
            if u.locked {
                exp_edges_in_mode += 1;
                if !exp_open {
                    exp_trim_sum += u.trim_mppb; // 閉窓のトリムを平均用に蓄積 (steady 周波数補正)
                    exp_trim_n += 1;
                }
                if !exp_open && exp_edges_in_mode >= EXP_TON_EDGES {
                    exp_open = true;
                    exp_edges_in_mode = 0;
                    // 保持トリム = 閉窓平均。瞬時値はノイズ/位相応答を含み過補正するため平均で steady 値を取る。
                    exp_held_trim = if exp_trim_n > 0 {
                        exp_trim_sum / exp_trim_n as i64
                    } else {
                        u.trim_mppb
                    };
                    info!(
                        "LOOPEXP open toff={} held_trim={}",
                        EXP_TOFF_SCHEDULE[exp_toff_idx], exp_held_trim
                    );
                } else if exp_open && exp_edges_in_mode >= EXP_TOFF_SCHEDULE[exp_toff_idx] {
                    exp_open = false;
                    exp_edges_in_mode = 0;
                    exp_trim_sum = 0;
                    exp_trim_n = 0; // 次の閉窓で平均を取り直す
                    exp_toff_idx = (exp_toff_idx + 1) % EXP_TOFF_SCHEDULE.len();
                    info!("LOOPEXP closed recal");
                    // 開窓では実効 K を放置したので、閉じる入口で測り直す (間欠運用の作法)。
                    k_target = recal_k(
                        &mut sm,
                        &mut output,
                        &cfg_gp2,
                        &cfg_gp4,
                        k_target,
                        last_period,
                    )
                    .await;
                    last_x = None;
                    last_gen = C0_GEN.load(Ordering::Acquire);
                }
            } else {
                exp_edges_in_mode = 0;
            }
        } else if u.locked {
            edges_since_recal += 1;
            // 段ゲート: S4+ のみ recal を駆動 (recal_eff=RECAL_EDGES)。S2/S3 は recal_eff=u32::MAX で
            // 実質 off (この閾値に達することはない → holdover gap なし)。S5 では現行と同一 (const assert)。
            if edges_since_recal >= recal_eff(PRECISION_STAGE) {
                edges_since_recal = 0;
                info!(
                    "PHASE_K recal start (after {} locked edges)",
                    recal_eff(PRECISION_STAGE)
                );
                // 測定値は k_target へ。実際の k は通常ループで毎エッジ slew して寄せる (段差回避)。
                // dk cap は前回測定値 (k_target) 比で評価 (measured-to-measured の真ドリフト)。
                k_target = recal_k(
                    &mut sm,
                    &mut output,
                    &cfg_gp2,
                    &cfg_gp4,
                    k_target,
                    last_period,
                )
                .await;
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

    // 温度FF の runtime トグルは段ゲート (temp_ff(stage))。S5=production で TEMP_FF_ENABLE と一致
    // (const assert で保証)。gen_capture_task / naive 経路が毎エッジ update_temp で温度を供給するが、
    // temp_ff_enable が false の間は predicted_freq は不変。
    CLOCK.lock(|g| {
        let mut g = g.borrow_mut();
        g.set_temp_ff_enable(temp_ff(PRECISION_STAGE));
        g.set_temp_ff_params(TEMP_FF_SM_SHIFT, TEMP_FF_SHIFT, TEMP_FF_GAIN_Q8);
        g.set_temp_ff_lag(TEMP_FF_LAG_Q8, TEMP_FF_DLEAD_SHIFT, RESID_OBS_SHIFT);
    });

    // 高優先度割込エグゼキュータは分岐の外で 1 回だけ起動する (PIO 経路 / naive 経路の双方がここへ spawn)。
    interrupt::SWI_IRQ_0.set_priority(Priority::P0);
    let spawner_high = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_0);

    // boot 設定行 (採用#4): 文書はこの 1 行を一次情報にする (コメントの古い結論に依拠しない)。新規
    // PPSCFG 行なので既存 regex 非干渉。i_den/d_den/smith は production slot0 (CTRL_GRID[SWEEP_SCHEDULE[0]])。
    // 補足: PPSGEN 末尾の dith_ticks は naive 出力が実適用した dither 周期 (embassy tick)。S1 は公称±1 tick
    // で揺れ、そのヒストグラムが sigma-delta の sub-tick 平均周波数の実機証跡。S0/PIO 経路は固定値 (不活性=0/公称)。
    {
        let cfg0 = CTRL_GRID[SWEEP_SCHEDULE[0] % CONTROLLER_LIST_LEN];
        let ctrl_name: &str = if !use_pio(PRECISION_STAGE) {
            "naive"
        } else if phase_servo(PRECISION_STAGE) {
            "pll"
        } else {
            "open_loop"
        };
        let exp_flags: u8 = (CONTROLLER_SWEEP as u8)
            | ((PRBS_INJECT as u8) << 1)
            | ((KICK_INJECT as u8) << 2)
            | ((INTERMITTENT_EXP as u8) << 3);
        info!(
            "PPSCFG stage={} use_pio={} apply_freq={} phase_servo={} recal_eff={} temp_ff={} ctrl={=str} i_den={} d_den={} smith={} phase_use_hw={} exp_flags={}",
            PRECISION_STAGE as u32,
            use_pio(PRECISION_STAGE) as u8,
            apply_freq(PRECISION_STAGE) as u8,
            phase_servo(PRECISION_STAGE) as u8,
            recal_eff(PRECISION_STAGE),
            temp_ff(PRECISION_STAGE) as u8,
            ctrl_name,
            cfg0.0,
            cfg0.1,
            cfg0.3,
            PHASE_USE_HW as u8,
            exp_flags
        );
    }

    if use_pio(PRECISION_STAGE) {
        // PPS on GP2 を PIO0 SM0 で rp-pps の PpsCapture でハードキャプチャ (~16ns 分解能)。
        let Pio {
            mut common,
            sm0,
            sm1,
            mut sm2,
            mut sm3,
            ..
        } = Pio::new(p.PIO0, Irqs);
        // 捕捉プログラムの variant 選択。比較し合う全 capture SM (SM0/SM2/SM3) を同じ variant で載せる。
        let capture_prog = if WRAP_BALANCED_CAPTURE {
            rp_pps::pps_capture_program_wrap_balanced()
        } else {
            rp_pps::pps_capture_program()
        };
        let mut capture = TimedPpsCapture::new_with_program(
            &mut common,
            sm0,
            p.PIN_2,
            clk_sys_freq(),
            &capture_prog,
        );

        // stage②: SM2 (ループバック位相計測。rp-pps 外の実験機能) は capture と GP2 を共有して生カウンタ差
        // K=C0−C2 を較正する。embassy の set_jmp_pin は &Pin を要求し GP2 は一度しか make できないので、
        // capture.jmp_pin() で GP2 Pin を借り、capture プログラムをもう一枚 load する。
        let lb_loaded = common.load_program(&capture_prog);
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

        // SM2 を GP4 (ループバック) に切替。jmp_pin レジスタだけ書く (set_config は exec_jmp(origin) で走行中
        // SM の X に −2 tick の段を作る。これは K 測定の後なので K に取り込まれず、ピン上の +32ns 固定オフセットに
        // なっていた。KPOKE 実験 20260704 で実証)。cfg_gp4 は recal_k / gen_capture_task へ move するので生成は残す。
        // enable 前に set_config(&cfg_gp2) 済み (program/clkdiv/pindirs は cfg_gp4 と同一) なので jmp_pin 切替で足りる。
        let mut cfg_gp4 = PioConfig::default();
        cfg_gp4.use_program(&lb_loaded, &[]);
        cfg_gp4.set_jmp_pin(&lb_pin);
        switch_jmp_pin(2, cfg_gp4.get_exec().jmp_pin);

        // SM1: 規律 PPS 生成を GP3 へ (rp-pps の SteeredPpsOutput)。初期周期 = ppb=0 の 1Hz (内部で
        // output_period_cycles)。周期は gen_capture_task が毎エッジ set_next_period で操舵する
        // (dither+period word 計算は rp-pps、servo=PLL は firmware 側に残す)。high 幅 100ms = GPS
        // モジュール/GPSDO の一般的な 1PPS 幅 (外部機器へ配れる; 規律対象は立ち上がりエッジで幅に不依存)。
        // 注 (20260704): この幅はカウンタのズレの実ドリフト源でもある。捕捉プログラムの low ループは X wrap
        // (68.7s 毎) で +1cy 損し high ループは損しないため、ピンの high-duty 差 (GPS-R PPS は high 900ms/low
        // 100ms 実測、出力は high 100ms) の分だけ 2 カウンタが 0.436×Δduty tick/min で離れる (=定期校正が
        // 追っていた ~5.6ns/min の正体)。900ms にして duty を揃えると march は −0.01ns/min に消える (実証済み)。
        // 本番は外部機器互換で 100ms のまま、恒久修正は WRAP_BALANCED_CAPTURE (一周コスト対称の capture) が担う。
        const PPS_PULSE_NS: u32 = 900_000_000; // 既定 = 幅合わせ (low 100ms、GPS-R と同じ)。外部機器互換が要る場合は 100ms へ
        let output = SteeredPpsOutput::new(&mut common, sm1, p.PIN_3, clk_sys_freq(), PPS_PULSE_NS);

        // --- 実験: SM3 を GP2 に常時向けた純観測 SM として起動 (KEXP 診断。制御には非関与) ---
        // boot K 較正・verify・cfg_gp4 切替が全部終わった後・spawn 直前に enable する (late enable):
        // 較正の繊細な c0/c2 read に干渉しえず、SM3 の enabled 全期間で consumer は pps_task 1 個に保たれる。
        // 既存 cfg_gp2 (jmp_pin=GP2, lb_loaded 再利用) をそのまま借りる。set_config は &Config を借りるだけで
        // この後 cfg_gp2 が gen_capture_task へ move されても借用は set_config で終わるので衝突しない。
        // GP2 の pindirs は SM0 が設定済み → set_pin_dirs 不要。X は初期化しない (絶対値は無意味、解析は
        // ドリフトのみ)。分周 default (=SM0/SM2 と同一レート)。
        // C3_WATCH_GP4 (偽捕捉仮説の検証): SM3 を GP4 へ向ける。enable 前なので set_config でよい。
        // cfg_gp4 は jmp_pin 以外 cfg_gp2 と同一、set_config は borrow のみでこの後の move と衝突しない。
        sm3.set_config(if C3_WATCH_GP4 { &cfg_gp4 } else { &cfg_gp2 }); // 命令メモリは lb_loaded 参照共有で増ゼロ
        sm3.set_enable(true);
        // enable 直後に SM0/SM3 両 GP2-FIFO を drain して等化 (SM0=verify のエッジ残留, SM3=enable 時に
        // GP2 が high なら入る spurious 捕捉を掃く → spawn 後の最初のエッジから c0/c3 が 1:1)。
        while capture.capture_mut().try_read().is_some() {}
        while sm3.rx().try_pull().is_some() {}
        info!("CAPCFG wrap_balanced={}", WRAP_BALANCED_CAPTURE as u8);
        if C3_WATCH_GP4 {
            info!("KEXPCFG sm3=gp4 prog=lb_loaded (pure observer, not in control)");
        } else {
            info!("KEXPCFG sm3=gp2 prog=lb_loaded (pure observer, not in control)");
        }

        // pps_task と gen_capture を高優先度割込エグゼキュータで起動 (ウェイクアップ遅延 ms→µs)。
        // cfg_gp2 は Copy なので gen_capture_task へも渡せる (SM2 の再較正切替に使う)。
        spawner_high.spawn(pps_task(capture, sm3).unwrap());
        spawner_high.spawn(gen_capture_task(output, sm2, k, cfg_gp2, cfg_gp4).unwrap());
    } else {
        // 非PIO 素朴経路 (S0/S1): PIO0 は未消費。GP2=GPS soft 割込、GP3=Ticker トグル、GP4 放置。
        // CLOCK (規律コア) は production と同一、front-end (TimedEdge を作る部分) だけが違う。
        let gps_in = Input::new(p.PIN_2, Pull::None);
        let pps_out = Output::new(p.PIN_3, Level::Low);
        spawner_high.spawn(naive_pps_in_task(gps_in).unwrap());
        spawner_high.spawn(naive_pps_out_task(pps_out).unwrap());
    }

    // 規律 UTC 出力タスクは両経路共通 (CLOCK をどちらの front-end が養っても動く)。
    spawner.spawn(time_task().unwrap());

    // 内蔵温度センサ (運用時の熱監視。規律には未使用、PPSGEN ログに temp_raw を出すだけ)。
    // DMA free-running で最大レート連続サンプリングする。DMA_CH0 は本 firmware 唯一の DMA 利用先。
    let adc = embassy_rp::adc::Adc::new(p.ADC, Irqs, embassy_rp::adc::Config::default());
    let ts = embassy_rp::adc::Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);
    let temp_dma = embassy_rp::dma::Channel::new(p.DMA_CH0, Irqs);
    spawner.spawn(temp_task(adc, ts, temp_dma).unwrap());

    // UART0: TX=GP0 (モジュール設定送信), RX=GP1 (NMEA 受信)。
    static TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let tx_buf = TX_BUF.init([0; 64]);
    let rx_buf = RX_BUF.init([0; 256]);
    let mut config = UartConfig::default();
    config.baudrate = mt3333::GNSS_BAUD;
    let mut uart = BufferedUart::new(p.UART0, p.PIN_0, p.PIN_1, Irqs, tx_buf, rx_buf, config);
    // 受信機のボーレートを引き上げてから分割する (set_baudrate は分割前の BufferedUart にしかない)。
    // 目的は帯域ではなく PPS↔NMEA 対応付けの余裕。詳細は mt3333::GNSS_FAST_BAUD の doc。
    mt3333::establish_link(&mut uart).await;
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
