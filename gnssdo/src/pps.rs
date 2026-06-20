//! PPS (pulse-per-second) エッジ列の追跡。
//!
//! GNSS の PPS は理想的には正確に 1Hz。受信した立ち上がりエッジの timestamp 列から、
//! ロック (≈1s)・パルス欠落・グリッチ/ジッタを判定する。どのライブラリも提供しない
//! アプリ固有ロジックなので、ここに置いて host で `cargo test-host` する。
//!
//! HAL 非依存: timestamp は単なるマイクロ秒の `u64`。firmware 側で
//! `embassy_time::Instant::now().as_micros()` を渡す。

use core::num::NonZeroU64;

/// PPS の公称間隔 (1 秒 = 1_000_000 us)。既定の 1Hz PPS 用。
pub const NOMINAL_US: u64 = 1_000_000;

/// ロック判定の既定許容誤差 (±50ms)。これを超えると欠落 or グリッチ扱い。
pub const TOLERANCE_US: u64 = 50_000;

/// [`PpsTracker`] の設定。1Hz 以外の PPS (例: 10Hz) や、受信機・捕捉系に応じた
/// 許容幅に対応するため設定可能にしている。`Default` は 1Hz・±50ms。
///
/// **注意**: `nominal_us` を 1Hz 以外にできるのは [`PpsTracker`] 単体のロック/欠落判定まで。
/// GPSDO の周波数規律 ([`crate::DisciplinedClock::update_freq`]) は現状 **1Hz 前提** (間隔を
/// 1e9 ns 基準で評価) なので、非 1Hz の `Locked` をそのまま周波数規律へ渡しても成立しない。
///
/// (`Debug`/`Default` は親 [`PpsTracker`] の derive が要求する最小限。)
#[derive(Debug)]
pub struct PpsTrackerConfig {
    /// PPS の公称間隔 (µs)。1Hz なら 1_000_000。`NonZeroU64` で 0 (ゼロ除算) を型排除。
    pub nominal_us: NonZeroU64,
    /// ロック判定の許容誤差 (µs)。公称 ± これ以内を Locked とする。欠落時は倍数ぶん広げる。
    /// 0 も有効 (公称ちょうどのみ Locked とする厳格判定) なので `NonZero` にはしない。
    pub tolerance_us: u64,
}

impl PpsTrackerConfig {
    /// 既定値 (1Hz, ±50ms)。`const fn` の [`PpsTracker::new`] から使えるよう const。
    pub const DEFAULT: Self = Self {
        nominal_us: NonZeroU64::new(NOMINAL_US).unwrap(),
        tolerance_us: TOLERANCE_US,
    };
}

impl Default for PpsTrackerConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// 1 エッジを記録したときの判定結果。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PpsEvent {
    /// 最初のエッジ (間隔は未確定)。
    First,
    /// 公称 1s ± 許容誤差。安定ロック。
    Locked { interval_us: u64 },
    /// 公称から外れた。`missed` は推定欠落パルス数 (グリッチ/ジッタなら 0)。
    Irregular { interval_us: u64, missed: u32 },
    /// 入力 timestamp が前回より過去だった (タイマ巻き戻り/順序逆転/リセット)。`backwards_us` は
    /// 戻った量。間隔が測れないので規律には使わない。内部状態は今回値へ rebase し次エッジから再開する。
    NonMonotonic { backwards_us: u64 },
}

/// PPS エッジ列の state machine。
#[derive(Debug, Default)]
pub struct PpsTracker {
    config: PpsTrackerConfig,
    last_us: Option<u64>,
    count: u32,
}

impl PpsTracker {
    /// 既定設定 (1Hz, ±50ms) で生成する。
    pub const fn new() -> Self {
        Self::with_config(PpsTrackerConfig::DEFAULT)
    }

    /// 設定を指定して生成する。`static` 初期化で使えるよう `const fn`。
    pub const fn with_config(config: PpsTrackerConfig) -> Self {
        Self {
            config,
            last_us: None,
            count: 0,
        }
    }

    /// 現在の設定。
    pub fn config(&self) -> &PpsTrackerConfig {
        &self.config
    }

    /// これまでに記録したエッジ総数。
    pub fn count(&self) -> u32 {
        self.count
    }

    /// 立ち上がりエッジを 1 つ記録し、判定結果を返す。
    /// `now_us` は単調増加するマイクロ秒タイムスタンプ。
    pub fn record(&mut self, now_us: u64) -> PpsEvent {
        self.count += 1;
        let nominal = self.config.nominal_us.get();
        let event = match self.last_us {
            None => PpsEvent::First,
            // 巻き戻り (タイマ wrap/順序逆転/リセット): 間隔が測れない。黙って 0 にせず報告する。
            Some(prev) if now_us < prev => PpsEvent::NonMonotonic {
                backwards_us: prev - now_us,
            },
            Some(prev) => {
                let interval = now_us - prev; // 上のガードで now_us >= prev
                // 公称間隔の何倍に最も近いか (四捨五入)。
                let n = (interval + nominal / 2) / nominal;
                if n >= 1 {
                    let expected = n * nominal;
                    let error = interval.abs_diff(expected);
                    // 倍数ぶんだけ許容誤差も広げる。
                    if error <= self.config.tolerance_us * n {
                        if n == 1 {
                            PpsEvent::Locked {
                                interval_us: interval,
                            }
                        } else {
                            PpsEvent::Irregular {
                                interval_us: interval,
                                missed: (n - 1) as u32,
                            }
                        }
                    } else {
                        PpsEvent::Irregular {
                            interval_us: interval,
                            missed: 0,
                        }
                    }
                } else {
                    // 0.5s 未満 = グリッチ。
                    PpsEvent::Irregular {
                        interval_us: interval,
                        missed: 0,
                    }
                }
            }
        };
        self.last_us = Some(now_us);
        event
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::num::NonZeroU64;

    #[test]
    fn non_monotonic_input_is_reported_and_rebases() {
        let mut t = PpsTracker::new();
        t.record(2_000_000);
        // 前回より過去 → 黙って 0 にせず NonMonotonic (戻り量 500ms) を返す。
        assert_eq!(
            t.record(1_500_000),
            PpsEvent::NonMonotonic {
                backwards_us: 500_000
            }
        );
        // 今回値へ rebase 済みなので、次の +1s は通常通り Locked に復帰する。
        assert_eq!(
            t.record(2_500_000),
            PpsEvent::Locked {
                interval_us: 1_000_000
            }
        );
        assert_eq!(t.count(), 3);
    }

    #[test]
    fn custom_tolerance_tightens_lock_window() {
        // 許容を ±5ms に絞ると、既定 (±50ms) では Locked だった +30ms ジッタが Irregular になる。
        let cfg = PpsTrackerConfig {
            tolerance_us: 5_000,
            ..PpsTrackerConfig::DEFAULT
        };
        let mut t = PpsTracker::with_config(cfg);
        t.record(0);
        assert_eq!(
            t.record(1_030_000),
            PpsEvent::Irregular {
                interval_us: 1_030_000,
                missed: 0
            }
        );
    }

    #[test]
    fn custom_nominal_supports_non_1hz() {
        // 公称 100ms (10Hz) でも 1 周期は Locked。1Hz 固定でない汎用性の確認。
        let cfg = PpsTrackerConfig {
            nominal_us: NonZeroU64::new(100_000).unwrap(),
            ..PpsTrackerConfig::DEFAULT
        };
        let mut t = PpsTracker::with_config(cfg);
        t.record(0);
        assert_eq!(
            t.record(100_000),
            PpsEvent::Locked {
                interval_us: 100_000
            }
        );
    }

    #[test]
    fn first_edge_is_first() {
        let mut t = PpsTracker::new();
        assert_eq!(t.record(5_000_000), PpsEvent::First);
        assert_eq!(t.count(), 1);
    }

    #[test]
    fn nominal_one_second_is_locked() {
        let mut t = PpsTracker::new();
        t.record(0);
        assert_eq!(
            t.record(NOMINAL_US),
            PpsEvent::Locked {
                interval_us: NOMINAL_US
            }
        );
        assert_eq!(t.count(), 2);
    }

    #[test]
    fn small_jitter_within_tolerance_is_locked() {
        let mut t = PpsTracker::new();
        t.record(0);
        // +30ms のジッタは許容内。
        assert_eq!(
            t.record(1_030_000),
            PpsEvent::Locked {
                interval_us: 1_030_000
            }
        );
    }

    #[test]
    fn one_missed_pulse() {
        let mut t = PpsTracker::new();
        t.record(0);
        // ~2s 空く = 1 パルス欠落。
        assert_eq!(
            t.record(2_000_000),
            PpsEvent::Irregular {
                interval_us: 2_000_000,
                missed: 1
            }
        );
    }

    #[test]
    fn three_missed_pulses() {
        let mut t = PpsTracker::new();
        t.record(0);
        assert_eq!(
            t.record(4_010_000),
            PpsEvent::Irregular {
                interval_us: 4_010_000,
                missed: 3
            }
        );
    }

    #[test]
    fn too_short_is_glitch() {
        let mut t = PpsTracker::new();
        t.record(0);
        // 0.2s = グリッチ (欠落ではない)。
        assert_eq!(
            t.record(200_000),
            PpsEvent::Irregular {
                interval_us: 200_000,
                missed: 0
            }
        );
    }

    #[test]
    fn off_nominal_between_multiples_is_glitch() {
        let mut t = PpsTracker::new();
        t.record(0);
        // 1.5s は 1 倍にも 2 倍にも遠い = ジッタ扱い (missed=0)。
        assert_eq!(
            t.record(1_500_000),
            PpsEvent::Irregular {
                interval_us: 1_500_000,
                missed: 0
            }
        );
    }

    #[test]
    fn lock_reacquires_after_gap() {
        let mut t = PpsTracker::new();
        t.record(0);
        t.record(2_000_000); // missed 1
        // 次が再び 1s ならロックに戻る。
        assert_eq!(
            t.record(3_000_000),
            PpsEvent::Locked {
                interval_us: 1_000_000
            }
        );
        assert_eq!(t.count(), 3);
    }
}
