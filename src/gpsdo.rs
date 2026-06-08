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

/// EMA の平滑係数 alpha = 1/2^EMA_SHIFT (≈ 時定数 32 サンプル)。
const EMA_SHIFT: u32 = 5;
/// ロック判定に必要な周波数サンプル数。
const LOCK_SAMPLES: u32 = 8;
/// 妥当な PPS 間隔範囲 (1s ± 1ms)。これより外れた間隔 (PIO ~68s 周回グリッチの偽短間隔
/// ≈ -37ms や欠落の ~2s) は周波数推定に使わない。50ms だと周回グリッチがすり抜けて
/// EMA を汚染した (実機評価で発覚)。真の間隔は 1s±数µs なので 1ms でも十分余裕がある。
const SANE_DEV_NS: i64 = 1_000_000;

/// PPS で規律されるクロックモデル。
#[derive(Debug, Default)]
pub struct DisciplinedClock {
    freq_mppb: i64, // 周波数オフセット EMA (milli-ppb = ppb*1000, 高分解能保持用)
    samples: u32,
    epoch_pio_ns: Option<u64>,     // PIO timebase のエポック (ns 精度, err/now_ns 用)
    epoch_instant_ns: Option<u64>, // Instant timebase のエポック (連続クエリ/holdover 用)
    epoch_unix_ns: Option<i64>,
    last_instant_ns: Option<u64>, // 最後に規律した Instant 時刻 (holdover 計測)
}

impl DisciplinedClock {
    pub const fn new() -> Self {
        Self {
            freq_mppb: 0,
            samples: 0,
            epoch_pio_ns: None,
            epoch_instant_ns: None,
            epoch_unix_ns: None,
            last_instant_ns: None,
        }
    }

    /// 精密な PPS 間隔 (ns, 理想 1e9) から周波数オフセットを EMA 更新する。
    /// 妥当範囲外 (wrap 等の外れ値) は無視。
    pub fn update_freq(&mut self, interval_ns: i64) {
        let dev = interval_ns - 1_000_000_000; // = ppb
        if dev.abs() >= SANE_DEV_NS {
            return;
        }
        let measured_mppb = dev * 1000;
        if self.samples == 0 {
            self.freq_mppb = measured_mppb;
        } else {
            // EMA: f += (x - f) / 2^SHIFT
            self.freq_mppb += (measured_mppb - self.freq_mppb) >> EMA_SHIFT;
        }
        self.samples += 1;
    }

    /// PPS エッジを UTC に対応付けてエポックを更新する。
    /// `pio_ns` = PIO の ns 精度時刻 (err/now_ns 用)、`instant_ns` = 連続して読める Instant 時刻
    /// (ticker/holdover 用)。両者は同じ XOSC 由来で同じ周波数オフセットを持つ。
    pub fn update_epoch(&mut self, pio_ns: u64, instant_ns: u64, unix_ns: i64) {
        self.epoch_pio_ns = Some(pio_ns);
        self.epoch_instant_ns = Some(instant_ns);
        self.epoch_unix_ns = Some(unix_ns);
        self.last_instant_ns = Some(instant_ns);
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
        self.samples >= LOCK_SAMPLES
    }

    /// **PIO timebase** の local 時刻 → 規律 UTC (Unix ns)。ns 精度。PPS エッジでの err 計測用。
    pub fn now_ns(&self, pio_ns: u64) -> Option<i64> {
        let ep = self.epoch_pio_ns?;
        let eu = self.epoch_unix_ns?;
        Some(eu + self.corrected(pio_ns as i64 - ep as i64))
    }

    /// **Instant timebase** の local 時刻 → 規律 UTC。連続クエリ (ticker) 用。サブ秒は Instant の µs 精度。
    pub fn now_from_instant_ns(&self, instant_ns: u64) -> Option<i64> {
        let ei = self.epoch_instant_ns?;
        let eu = self.epoch_unix_ns?;
        Some(eu + self.corrected(instant_ns as i64 - ei as i64))
    }

    /// 逆変換: 指定した UTC が来る **Instant timebase** の local 時刻 (ns)。周波数補正込み。
    /// 「正確な UTC 時刻 T に何かを実行する」スケジューリングや、補正済みの待ち時間に使う。
    pub fn local_instant_for_unix_ns(&self, unix_ns: i64) -> Option<i64> {
        let ei = self.epoch_instant_ns? as i64;
        let eu = self.epoch_unix_ns?;
        let dt = unix_ns - eu; // 真の経過 (ns)
        // local 経過 = 真 / (1 - ppb/1e9) ≈ 真 + 真*ppb/1e9。mppb は ppb*1000。
        let d = dt + (dt as i128 * self.freq_mppb as i128 / 1_000_000_000_000i128) as i64;
        Some(ei + d)
    }

    /// 補正遅延の素: 「真の時間で `true_ns` 待つ」のに必要なローカルクロックの ns。
    /// ローカルは ppb 分速い/遅いので、その分だけ多く/少なく待つ。`Timer::after` に被せると
    /// 水晶公差でなく ±ppb の正確さで待てる (例: `after_micros(true_to_local_ns(us*1000)/1000)`)。
    pub fn true_to_local_ns(&self, true_ns: i64) -> i64 {
        true_ns + (true_ns as i128 * self.freq_mppb as i128 / 1_000_000_000_000i128) as i64
    }

    /// 最後に PPS で規律してからの経過 (holdover 時間, ns)。Instant timebase で渡す。
    pub fn holdover_ns(&self, instant_ns: u64) -> u64 {
        match self.last_instant_ns {
            Some(t) => instant_ns.saturating_sub(t),
            None => 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn not_locked_until_enough_samples() {
        let mut c = DisciplinedClock::new();
        for _ in 0..(LOCK_SAMPLES - 1) {
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
        assert_eq!(c.now_ns(1_500_000_000), Some(5_000_500_000_000));
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
        assert_eq!(c.now_ns(1_000_000_000), Some(1_000_000_000 - 100_000));
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
    fn now_is_none_before_epoch() {
        let c = DisciplinedClock::new();
        assert_eq!(c.now_ns(123), None);
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
        let local = c.local_instant_for_unix_ns(target).unwrap();
        let back = c.now_from_instant_ns(local as u64).unwrap();
        assert!((back - target).abs() <= 2, "roundtrip off by {}", back - target);
        // 補正が効いていれば local 経過 > 真の経過 (水晶が速いぶん先に進む): +3ppm×3s ≈ +9µs。
        assert!(local > 1_000_000_000 + 3_000_000_000);
    }
}
