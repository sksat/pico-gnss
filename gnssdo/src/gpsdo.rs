//! GPS 規律発振器 (GPSDO) のクロック規律。
//!
//! PPS 間隔 (PIO ハードキャプチャの ns 精度) から RP2040 水晶の周波数オフセットを EMA 推定し、
//! UTC エポックと合わせて「規律された UTC」を device 上で提供する。**PPS が切れている間
//! (holdover) も、推定した周波数で外挿して時刻を保つ**のが GPSDO の肝。
//!
//! 単位は ppb (parts per billion = ns/s)。PPS 間隔の偏差 (interval_ns − 1e9) がそのまま
//! 周波数オフセット ppb になる (1 ppb = 1 ns/s)。
//!
//! 周波数推定は PIO の精密な間隔から、エポック(絶対時刻の基点)は連続的に読める local clock
//! (embassy Instant 等) から、と別々に与える。両者は単位 (ns) を揃えるだけでよい。
//! いずれも HAL 非依存なので host で `cargo test-host` する。

use core::num::{NonZeroU32, NonZeroU64};

/// [`DisciplinedClock`] のチューニング設定。
///
/// 受信機・アンテナ・TCXO/水晶・PPS 捕捉分解能・運用形態 (固定/移動) で適値が変わるため、
/// 固定 const ではなく設定可能にしている。`Default` (= [`DisciplinedClockConfig::DEFAULT`]) は
/// GYSFFMANC (MT3333) + 窓際固定 + PIO ns 捕捉での実測整定値。
///
/// **不正値は型で排除する**: 0 や負だと意味を成さない/破綻するフィールド (サンプル数・各ゲート)
/// は `NonZero*` にしてあり、不正値はそもそも構築できない。シフト量だけは上限制約 (0..=63) なので
/// `NonZero` では表せず、使用時に `min(63)` でクランプする (i64 シフトのオーバーフロー回避)。
///
/// (`Debug`/`Default` は親 [`DisciplinedClock`] の derive が要求する最小限。
///  比較や複製が要るようになるまで `Clone`/`PartialEq` 等は足さない。)
#[derive(Debug)]
pub struct DisciplinedClockConfig {
    /// ロック後の EMA 平滑係数 alpha = 1/2^`ema_shift` (既定 5 ≈ 時定数 32 サンプル)。
    /// 有効範囲 0..=63、超過分は内部で 63 にクランプする。
    pub ema_shift: u32,
    /// 収束中 (未ロック) の速い EMA 係数 alpha = 1/2^`ema_shift_fast` (既定 3 = 1/8)。
    /// 起動直後はまだ推定が無いので速く捕捉し、ロック後はゆっくり平滑してジッタを抑える。
    /// 有効範囲 0..=63 (`ema_shift` と同様クランプ)。
    pub ema_shift_fast: u32,
    /// ロック判定に必要な周波数サンプル数 (既定 8)。
    pub lock_samples: NonZeroU32,
    /// **非常停止**枠 (1s ± `sane_dev_ns`, 既定 1ms)。これより外れた間隔 (PIO ~68s 周回
    /// グリッチの偽短間隔 ≈ -37ms や欠落の ~2s) は明らかに不正。50ms だと周回グリッチが
    /// すり抜けて EMA を汚染した (実機評価で発覚)。通常の品質判定ではなく最後の安全網であり、
    /// 通常品質は下の収束ゲート / 残差ゲートで弾く (smart-friend GPT-5.5 の指摘)。
    pub sane_dev_ns: NonZeroU64,
    /// 未ロック (収束中) の絶対品質ゲート (既定 ±100µs)。EMA がまだ信用できないので絶対値で弾く。
    /// 窓際弱信号の中規模 multipath (数十〜数百µs) が最初の数発に混じって EMA を汚すのを防ぐ。
    pub converge_gate_ns: NonZeroU64,
    /// ロック後の残差ゲート: `|measured − EMA|` が `residual_gate_ns` (既定 ±5µs) を超える単発は
    /// multipath とみなし棄却。固定窓際の真のジッタは ns〜数十ns 級なので 5µs でも十分甘い安全側。
    pub residual_gate_ns: NonZeroU64,
    /// holdover/Irregular から復帰した直後、周波数 EMA 更新を保留するサンプル数 (既定 5)。
    /// 復帰直後の PPS は受信機内部状態・NMEA 対応・PPS 位相がまだ整っておらず信用できない。
    /// 0 = 検疫しない (有効な設定なので `NonZero` にはしない)。
    pub quarantine_samples: u32,
}

impl DisciplinedClockConfig {
    /// 実測整定の既定値。`const fn` の [`DisciplinedClock::new`] から使えるよう const で保持。
    pub const DEFAULT: Self = Self {
        ema_shift: 5,
        ema_shift_fast: 3,
        lock_samples: NonZeroU32::new(8).unwrap(),
        sane_dev_ns: NonZeroU64::new(1_000_000).unwrap(),
        converge_gate_ns: NonZeroU64::new(100_000).unwrap(),
        residual_gate_ns: NonZeroU64::new(5_000).unwrap(),
        quarantine_samples: 5,
    };
}

impl Default for DisciplinedClockConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// `update_freq` の結果。pps_task 側のログ/状態管理に使う。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FreqUpdate {
    /// EMA を更新した。
    Applied,
    /// ±1ms 非常停止 (wrap グリッチ/欠落) で棄却。
    GatedSane,
    /// 品質ゲート (収束中の絶対 / ロック後の残差) で棄却。
    GatedQuality,
    /// 復帰検疫中につき EMA 更新を保留した。
    Quarantined,
}

/// PPS で規律されるクロックモデル。
///
/// 2 つの timebase を扱う (どちらも device の同じ発振器由来・整数 ns で渡す。HAL 非依存):
/// - **capture timebase**: PPS エッジを高分解能で捕捉する系。err 計測や `fire_at_utc` の ns 精度に使う。
///   RP2040 では PIO ハードキャプチャ (他チップではタイマ入力キャプチャ等)。
/// - **query timebase**: 連続して読める系。ticker/holdover に使う。RP2040 では embassy `Instant`。
///
/// 両系のエポックは [`update_epoch`](Self::update_epoch) で同時に与える。
#[derive(Debug, Default)]
pub struct DisciplinedClock {
    config: DisciplinedClockConfig,
    freq_mppb: i64, // 周波数オフセット EMA (milli-ppb = ppb*1000, 高分解能保持用)
    samples: u32,
    quarantine: u32, // >0 の間は復帰検疫中 (周波数 EMA 更新を保留)
    epoch_capture_ns: Option<u64>,     // capture timebase のエポック (ns 精度, err/now_from_capture_ns 用)
    epoch_query_ns: Option<u64>, // query timebase のエポック (連続クエリ/holdover 用)
    epoch_unix_ns: Option<i64>,
    last_query_ns: Option<u64>, // 最後に規律した Instant 時刻 (holdover 計測)
}

/// 整数秒のズレを除いて sub 秒の残差だけ残す。
/// 補正後の予測残差 (err) は、PPS が複数秒途切れた後の復帰時に PPS↔RMC のペアリングが整数秒
/// ズレることがあり raw だと ~Ns の巨大値になる。最寄りの整数秒へ snap すると、その中に埋もれた
/// **真の holdover 残差** (sub 秒) が取り出せる (例: 25_000_000_360 → 360ns = 25s holdover の誤差)。
pub fn snap_to_second_ns(raw: i64) -> i64 {
    let secs = (raw + raw.signum() * 500_000_000) / 1_000_000_000;
    raw - secs * 1_000_000_000
}

impl DisciplinedClock {
    /// 既定設定 ([`DisciplinedClockConfig::DEFAULT`]) で生成する。
    pub const fn new() -> Self {
        Self::with_config(DisciplinedClockConfig::DEFAULT)
    }

    /// 設定を指定して生成する。`static` 初期化で使えるよう `const fn`。
    pub const fn with_config(config: DisciplinedClockConfig) -> Self {
        Self {
            config,
            freq_mppb: 0,
            samples: 0,
            quarantine: 0,
            epoch_capture_ns: None,
            epoch_query_ns: None,
            epoch_unix_ns: None,
            last_query_ns: None,
        }
    }

    /// 現在の設定。
    pub fn config(&self) -> &DisciplinedClockConfig {
        &self.config
    }

    /// 精密な PPS 間隔 (ns, 理想 1e9) から周波数オフセットを EMA 更新する。
    ///
    /// 多段ゲートで弱信号 (窓際 multipath)・wrap グリッチ・復帰直後の怪しい PPS を弾く:
    /// 1. **非常停止** ±1ms 外 → 棄却 (wrap/欠落)。
    /// 2. **復帰検疫** 中 → 更新せず保留 (過去の推定の方が信用できる)。
    /// 3. **品質ゲート** 未ロックは絶対 ±100µs、ロック後は EMA 残差 ±5µs。
    /// 4. EMA 更新 (収束中は速い alpha、ロック後はゆっくり)。
    ///
    /// **呼ぶ側の責務**: `PpsTracker` が `Locked` と判定したエッジでのみ呼ぶこと
    /// (Irregular/First の間隔は周波数推定に使わない)。復帰時は `start_quarantine()`。
    pub fn update_freq(&mut self, interval_ns: i64) -> FreqUpdate {
        let dev = interval_ns - 1_000_000_000; // = ppb
        // 1. 非常停止: ±1ms 外は wrap グリッチ/欠落。常に棄却。
        if dev.abs() >= self.config.sane_dev_ns.get() as i64 {
            return FreqUpdate::GatedSane;
        }
        // 2. 復帰検疫: 復帰直後 M サンプルは受信機内部状態・PPS 位相が未整合で信用できない
        //    ので EMA を更新せず、過去の水晶周波数推定を保つ。
        if self.quarantine > 0 {
            self.quarantine -= 1;
            return FreqUpdate::Quarantined;
        }
        let measured_mppb = dev * 1000;
        // 最初の 1 発はゲート基準 (EMA) が無いので、非常停止内ならそのまま採用。
        if self.samples == 0 {
            self.freq_mppb = measured_mppb;
            self.samples = 1;
            return FreqUpdate::Applied;
        }
        // 3. 品質ゲート。
        if self.is_locked() {
            // ロック後: EMA からの残差が大きい単発は multipath。±1ms より遥かに厳しく弾く。
            if (measured_mppb - self.freq_mppb).abs() > self.config.residual_gate_ns.get() as i64 * 1000 {
                return FreqUpdate::GatedQuality;
            }
        } else {
            // 収束中: EMA がまだ信用できないので絶対ゲートで中規模 multipath を弾く。
            if dev.abs() > self.config.converge_gate_ns.get() as i64 {
                return FreqUpdate::GatedQuality;
            }
        }
        // 4. EMA 更新: f += (x − f) / 2^SHIFT。収束中は速い alpha=1/8 で捕捉。
        // シフト量は 0..=63 にクランプして i64 シフトのオーバーフローを防ぐ (config doc 参照)。
        let shift = if self.is_locked() {
            self.config.ema_shift
        } else {
            self.config.ema_shift_fast
        }
        .min(63);
        self.freq_mppb += (measured_mppb - self.freq_mppb) >> shift;
        self.samples += 1;
        FreqUpdate::Applied
    }

    /// holdover/Irregular から `Locked` へ復帰したとき呼ぶ。直後 `QUARANTINE_SAMPLES` 発の
    /// 周波数 EMA 更新を保留する (EMA リセットはしない — 短断なら過去推定の方が信用できる)。
    /// 守るべき推定がまだ無い初回捕捉 (samples==0) では何もしない — 起動時の捕捉を遅らせない。
    pub fn start_quarantine(&mut self) {
        if self.samples > 0 {
            self.quarantine = self.config.quarantine_samples;
        }
    }

    /// 復帰検疫中か (ログ/デバッグ用)。
    pub fn in_quarantine(&self) -> bool {
        self.quarantine > 0
    }

    /// PPS エッジを UTC に対応付けてエポックを更新する。
    /// `capture_ns` = PIO の ns 精度時刻 (err/now_from_capture_ns 用)、`query_ns` = 連続して読める Instant 時刻
    /// (ticker/holdover 用)。両者は同じ XOSC 由来で同じ周波数オフセットを持つ。
    pub fn update_epoch(&mut self, capture_ns: u64, query_ns: u64, unix_ns: i64) {
        self.epoch_capture_ns = Some(capture_ns);
        self.epoch_query_ns = Some(query_ns);
        self.epoch_unix_ns = Some(unix_ns);
        self.last_query_ns = Some(query_ns);
    }

    /// local 経過 d (ns) を周波数補正: 真の経過 = d - d*ppb/1e9 = d - d*mppb/1e12。
    fn corrected(&self, d: i64) -> i64 {
        d - (d as i128 * self.freq_mppb as i128 / 1_000_000_000_000i128) as i64
    }

    /// 推定周波数オフセット (ppb)。
    pub fn freq_ppb(&self) -> i64 {
        let half = 500 * self.freq_mppb.signum();
        (self.freq_mppb + half) / 1000
    }

    /// 推定周波数オフセット (milli-ppb)。webapp で ppm 表示する用。
    pub fn freq_mppb(&self) -> i64 {
        self.freq_mppb
    }

    /// 十分なサンプルで周波数がロックしたか。
    pub fn is_locked(&self) -> bool {
        self.samples >= self.config.lock_samples.get()
    }

    /// **capture timebase** の local 時刻 → 規律 UTC (Unix ns)。ns 精度。PPS エッジでの err 計測用。
    pub fn now_from_capture_ns(&self, capture_ns: u64) -> Option<i64> {
        let ep = self.epoch_capture_ns?;
        let eu = self.epoch_unix_ns?;
        Some(eu + self.corrected(capture_ns as i64 - ep as i64))
    }

    /// **query timebase** の local 時刻 → 規律 UTC。連続クエリ (ticker) 用。サブ秒は Instant の µs 精度。
    pub fn now_from_query_ns(&self, query_ns: u64) -> Option<i64> {
        let ei = self.epoch_query_ns?;
        let eu = self.epoch_unix_ns?;
        Some(eu + self.corrected(query_ns as i64 - ei as i64))
    }

    /// 逆変換: 指定した UTC が来る **query timebase** の local 時刻 (ns)。周波数補正込み。
    /// 「正確な UTC 時刻 T に何かを実行する」スケジューリングや、補正済みの待ち時間に使う。
    pub fn query_ns_for_unix_ns(&self, unix_ns: i64) -> Option<i64> {
        let ei = self.epoch_query_ns? as i64;
        let eu = self.epoch_unix_ns?;
        let dt = unix_ns - eu; // 真の経過 (ns)
        // local 経過 = 真 / (1 - ppb/1e9) ≈ 真 + 真*ppb/1e9。mppb は ppb*1000。
        let d = dt + (dt as i128 * self.freq_mppb as i128 / 1_000_000_000_000i128) as i64;
        Some(ei + d)
    }

    /// 逆変換: 指定した UTC が来る **capture timebase** の local 時刻 (ns)。`now_from_capture_ns` の逆。
    /// `fire_at_utc(T)` の核 — この値を PIO の生成/比較 SM に目標 tick として渡せば、UTC ちょうど T に
    /// ピンを駆動できる。捕捉と同じ capture timebase なので ns 精度。
    pub fn capture_ns_for_unix_ns(&self, unix_ns: i64) -> Option<i64> {
        let ep = self.epoch_capture_ns? as i64;
        let eu = self.epoch_unix_ns?;
        let dt = unix_ns - eu; // 真の経過 (ns)
        let d = dt + (dt as i128 * self.freq_mppb as i128 / 1_000_000_000_000i128) as i64;
        Some(ep + d)
    }

    /// 補正遅延の素: 「真の時間で `true_ns` 待つ」のに必要なローカルクロックの ns。
    /// ローカルは ppb 分速い/遅いので、その分だけ多く/少なく待つ。`Timer::after` に被せると
    /// 水晶公差でなく ±ppb の正確さで待てる (例: `after_micros(true_to_local_ns(us*1000)/1000)`)。
    pub fn true_to_local_ns(&self, true_ns: i64) -> i64 {
        true_ns + (true_ns as i128 * self.freq_mppb as i128 / 1_000_000_000_000i128) as i64
    }

    /// 最後に PPS で規律してからの経過 (holdover 時間, ns)。query timebase で渡す。
    pub fn holdover_ns(&self, query_ns: u64) -> u64 {
        match self.last_query_ns {
            Some(t) => query_ns.saturating_sub(t),
            None => 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::num::{NonZeroU32, NonZeroU64};

    #[test]
    fn custom_config_changes_lock_threshold() {
        // lock_samples を 3 に下げると 3 サンプルでロックする (既定 8 とは異なる)。
        let cfg = DisciplinedClockConfig {
            lock_samples: NonZeroU32::new(3).unwrap(),
            ..DisciplinedClockConfig::DEFAULT
        };
        let mut c = DisciplinedClock::with_config(cfg);
        c.update_freq(1_000_002_500);
        c.update_freq(1_000_002_500);
        assert!(!c.is_locked());
        c.update_freq(1_000_002_500);
        assert!(c.is_locked());
    }

    #[test]
    fn custom_converge_gate_admits_wider_outlier() {
        // converge_gate_ns を広げると、既定 (±100µs) では弾かれる偏差を収束中でも採用する。
        let cfg = DisciplinedClockConfig {
            converge_gate_ns: NonZeroU64::new(500_000).unwrap(),
            ..DisciplinedClockConfig::DEFAULT
        };
        let mut c = DisciplinedClock::with_config(cfg);
        c.update_freq(1_000_002_500); // 基準確立
        assert_eq!(c.update_freq(1_000_000_000 + 200_000), FreqUpdate::Applied);
    }

    #[test]
    fn freq_converges_to_constant_offset() {
        let mut c = DisciplinedClock::new();
        for _ in 0..40 {
            c.update_freq(1_000_002_500); // +2500 ppb (= +2.5 ppm)
        }
        assert_eq!(c.freq_ppb(), 2500);
        assert!(c.is_locked());
    }

    #[test]
    fn freq_ema_smooths_jitter() {
        let mut c = DisciplinedClock::new();
        // 2500 を中心に ±16ns で振れる入力 → 平滑後はほぼ 2500。
        for i in 0..200 {
            let j = if i % 2 == 0 { 16 } else { -16 };
            c.update_freq(1_000_000_000 + 2500 + j);
        }
        assert!((c.freq_ppb() - 2500).abs() <= 20, "freq={}", c.freq_ppb());
    }

    #[test]
    fn outliers_ignored() {
        let mut c = DisciplinedClock::new();
        c.update_freq(1_000_002_500);
        c.update_freq(500_000_000); // wrap 由来の外れ値 → 無視
        c.update_freq(2_000_000_000); // 同上
        assert_eq!(c.freq_ppb(), 2500);
        assert_eq!(c.samples, 1);
    }

    #[test]
    fn update_freq_returns_applied_or_gated() {
        let mut c = DisciplinedClock::new();
        assert_eq!(c.update_freq(1_000_002_500), FreqUpdate::Applied);
        assert_eq!(c.update_freq(500_000_000), FreqUpdate::GatedSane); // 非常停止
    }

    #[test]
    fn pre_lock_converge_gate_rejects_midsize_outlier() {
        let mut c = DisciplinedClock::new();
        c.update_freq(1_000_002_500); // 1 発目で基準確立 (+2500ppb)
        // 未ロック中、±1ms 内だが ±100µs を超える中規模 multipath は弾く。
        assert_eq!(c.update_freq(1_000_000_000 + 200_000), FreqUpdate::GatedQuality);
        assert_eq!(c.samples, 1);
        assert_eq!(c.freq_ppb(), 2500); // 汚染されていない
    }

    #[test]
    fn post_lock_residual_gate_rejects_multipath() {
        let mut c = DisciplinedClock::new();
        for _ in 0..12 {
            c.update_freq(1_000_002_500); // +2500ppb でロック
        }
        assert!(c.is_locked());
        // ロック後、EMA から +10µs ずれた単発 (multipath) は残差ゲートで棄却。
        assert_eq!(c.update_freq(1_000_000_000 + 2500 + 10_000), FreqUpdate::GatedQuality);
        assert_eq!(c.freq_ppb(), 2500); // EMA は動かない
        // EMA 近傍 (±数十ns ジッタ) は通る。
        assert_eq!(c.update_freq(1_000_002_500 + 16), FreqUpdate::Applied);
    }

    #[test]
    fn quarantine_holds_updates_after_recovery() {
        let mut c = DisciplinedClock::new();
        for _ in 0..12 {
            c.update_freq(1_000_002_500); // ロック (+2500ppb)
        }
        c.start_quarantine();
        assert!(c.in_quarantine());
        // 復帰直後 5 発は (たとえ妥当範囲でも) EMA を更新しない。
        for _ in 0..5 {
            assert_eq!(c.update_freq(1_000_005_000), FreqUpdate::Quarantined); // +5000ppb の怪しい PPS
        }
        assert_eq!(c.freq_ppb(), 2500); // 検疫が EMA を守った
        assert!(!c.in_quarantine());
        // 検疫明けは通常通り更新する。
        assert_eq!(c.update_freq(1_000_002_500), FreqUpdate::Applied);
    }

    #[test]
    fn quarantine_noop_on_cold_boot() {
        let mut c = DisciplinedClock::new();
        c.start_quarantine(); // まだ推定が無い → 検疫しない
        assert!(!c.in_quarantine());
        // 初回捕捉はすぐ採用される (起動時の捕捉が遅れない)。
        assert_eq!(c.update_freq(1_000_002_500), FreqUpdate::Applied);
        assert_eq!(c.freq_ppb(), 2500);
    }

    #[test]
    fn fast_alpha_converges_within_few_samples() {
        let mut c = DisciplinedClock::new();
        // 未ロックの速い alpha=1/8 で、定常オフセットに数発で寄る。
        for _ in 0..DisciplinedClockConfig::DEFAULT.lock_samples.get() {
            c.update_freq(1_000_003_000); // +3000ppb
        }
        // 1 発目で 3000 に張り付くので、ロック時点で既にほぼ 3000。
        assert!((c.freq_ppb() - 3000).abs() <= 50, "freq={}", c.freq_ppb());
    }

    #[test]
    fn not_locked_until_enough_samples() {
        let mut c = DisciplinedClock::new();
        for _ in 0..(DisciplinedClockConfig::DEFAULT.lock_samples.get() - 1) {
            c.update_freq(1_000_002_500);
        }
        assert!(!c.is_locked());
        c.update_freq(1_000_002_500);
        assert!(c.is_locked());
    }

    #[test]
    fn now_without_correction() {
        let mut c = DisciplinedClock::new();
        c.update_epoch(1_000_000_000, 1_000_000_000, 5_000_000_000_000);
        // freq=0 → 補正なし。0.5s 後。
        assert_eq!(c.now_from_capture_ns(1_500_000_000), Some(5_000_500_000_000));
    }

    #[test]
    fn now_applies_freq_correction_during_holdover() {
        let mut c = DisciplinedClock::new();
        // +100,000 ppb = +100 ppm (誇張) で補正を見やすく (フィルタ ±1ms 内に収める)。
        for _ in 0..40 {
            c.update_freq(1_000_000_000 + 100_000);
        }
        assert_eq!(c.freq_ppb(), 100_000);
        c.update_epoch(0, 0, 0);
        // local 経過 1e9 ns。真の経過 = 1e9 - 1e9*1e5/1e12 = 1e9 - 1e5。
        assert_eq!(c.now_from_capture_ns(1_000_000_000), Some(1_000_000_000 - 100_000));
    }

    #[test]
    fn holdover_counts_since_last_epoch() {
        let mut c = DisciplinedClock::new();
        c.update_epoch(1_000_000_000, 1_000_000_000, 0);
        assert_eq!(c.holdover_ns(1_000_000_000), 0);
        assert_eq!(c.holdover_ns(4_000_000_000), 3_000_000_000); // 3s holdover
    }

    #[test]
    fn true_to_local_applies_offset() {
        let mut c = DisciplinedClock::new();
        // freq=0 → 補正なし (恒等)。
        assert_eq!(c.true_to_local_ns(1_000_000_000), 1_000_000_000);
        for _ in 0..40 {
            c.update_freq(1_000_000_000 + 100_000); // +100 ppm
        }
        // ローカルは速いので、真の 1s 待つには +100µs 多くローカルで待つ。
        assert_eq!(c.true_to_local_ns(1_000_000_000), 1_000_000_000 + 100_000);
    }

    #[test]
    fn snap_recovers_subsecond_residual() {
        // 通常の小さい残差はそのまま。
        assert_eq!(snap_to_second_ns(360), 360);
        assert_eq!(snap_to_second_ns(-41), -41);
        // 1 秒・25 秒の整数ズレに埋もれた残差を取り出す。
        assert_eq!(snap_to_second_ns(1_000_000_041), 41);
        assert_eq!(snap_to_second_ns(25_000_000_360), 360);
        assert_eq!(snap_to_second_ns(-1_000_000_041), -41);
        // 秒境界の手前 (1s より 100ns 手前) は -100ns。
        assert_eq!(snap_to_second_ns(999_999_900), -100);
    }

    #[test]
    fn now_is_none_before_epoch() {
        let c = DisciplinedClock::new();
        assert_eq!(c.now_from_capture_ns(123), None);
    }

    #[test]
    fn local_for_unix_roundtrips_with_now() {
        let mut c = DisciplinedClock::new();
        for _ in 0..40 {
            c.update_freq(1_000_000_000 + 3000); // +3 ppm (現実的な水晶オフセット)
        }
        assert_eq!(c.freq_ppb(), 3000);
        c.update_epoch(1_000_000_000, 1_000_000_000, 5_000_000_000_000);
        // 3 秒後の UTC が来る local 時刻を求め、その local で now すれば元の UTC に戻る (Instant 系)。
        let target = 5_000_000_000_000 + 3_000_000_000;
        let local = c.query_ns_for_unix_ns(target).unwrap();
        let back = c.now_from_query_ns(local as u64).unwrap();
        assert!((back - target).abs() <= 2, "roundtrip off by {}", back - target);
        // 補正が効いていれば local 経過 > 真の経過 (水晶が速いぶん先に進む): +3ppm×3s ≈ +9µs。
        assert!(local > 1_000_000_000 + 3_000_000_000);
    }

    #[test]
    fn pio_local_for_unix_roundtrips_with_now() {
        // fire_at_utc の核: UTC → PIO tick の逆変換が now_from_capture_ns (PIO tick → UTC) と往復一致する。
        let mut c = DisciplinedClock::new();
        for _ in 0..40 {
            c.update_freq(1_000_000_000 + 3000); // +3 ppm
        }
        c.update_epoch(1_000_000_000, 1_000_000_000, 5_000_000_000_000);
        let target = 5_000_000_000_000 + 3_000_000_000; // 3 秒後の UTC
        let tick = c.capture_ns_for_unix_ns(target).unwrap(); // この PIO tick でピンを駆動すれば UTC=target
        let back = c.now_from_capture_ns(tick as u64).unwrap();
        assert!((back - target).abs() <= 2, "roundtrip off by {}", back - target);
        // 水晶が +3ppm 速いので、3 秒先の UTC に対し PIO tick は真の経過より先 (+9µs)。
        assert!(tick > 1_000_000_000 + 3_000_000_000);
    }

    #[test]
    fn pio_local_for_unix_none_before_epoch() {
        let c = DisciplinedClock::new();
        assert_eq!(c.capture_ns_for_unix_ns(5_000_000_000_000), None);
    }

    #[test]
    fn pio_local_for_unix_no_freq_offset_is_identity_shift() {
        // freq=0 なら PIO tick = epoch_pio + (unix - epoch_unix) (補正なし)。
        let mut c = DisciplinedClock::new();
        c.update_epoch(1_000_000_000, 1_000_000_000, 5_000_000_000_000);
        assert_eq!(
            c.capture_ns_for_unix_ns(5_000_000_500_000),
            Some(1_000_000_000 + 500_000)
        );
    }
}
