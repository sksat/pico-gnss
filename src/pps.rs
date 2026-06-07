//! PPS (pulse-per-second) エッジ列の追跡。
//!
//! GNSS の PPS は理想的には正確に 1Hz。受信した立ち上がりエッジの timestamp 列から、
//! ロック (≈1s)・パルス欠落・グリッチ/ジッタを判定する。どのライブラリも提供しない
//! アプリ固有ロジックなので、ここに置いて host で `cargo test-host` する。
//!
//! HAL 非依存: timestamp は単なるマイクロ秒の `u64`。firmware 側で
//! `embassy_time::Instant::now().as_micros()` を渡す。

/// PPS の公称間隔 (1 秒 = 1_000_000 us)。
pub const NOMINAL_US: u64 = 1_000_000;

/// ロック判定の許容誤差 (±50ms)。これを超えると欠落 or グリッチ扱い。
pub const TOLERANCE_US: u64 = 50_000;

/// 1 エッジを記録したときの判定結果。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PpsEvent {
    /// 最初のエッジ (間隔は未確定)。
    First,
    /// 公称 1s ± 許容誤差。安定ロック。
    Locked { interval_us: u64 },
    /// 公称から外れた。`missed` は推定欠落パルス数 (グリッチ/ジッタなら 0)。
    Irregular { interval_us: u64, missed: u32 },
}

/// PPS エッジ列の state machine。
#[derive(Debug, Default)]
pub struct PpsTracker {
    last_us: Option<u64>,
    count: u32,
}

impl PpsTracker {
    pub const fn new() -> Self {
        Self {
            last_us: None,
            count: 0,
        }
    }

    /// これまでに記録したエッジ総数。
    pub fn count(&self) -> u32 {
        self.count
    }

    /// 立ち上がりエッジを 1 つ記録し、判定結果を返す。
    /// `now_us` は単調増加するマイクロ秒タイムスタンプ。
    pub fn record(&mut self, now_us: u64) -> PpsEvent {
        self.count += 1;
        let event = match self.last_us {
            None => PpsEvent::First,
            Some(prev) => {
                let interval = now_us.saturating_sub(prev);
                // 公称間隔の何倍に最も近いか (四捨五入)。
                let n = (interval + NOMINAL_US / 2) / NOMINAL_US;
                if n >= 1 {
                    let expected = n * NOMINAL_US;
                    let error = interval.abs_diff(expected);
                    // 倍数ぶんだけ許容誤差も広げる。
                    if error <= TOLERANCE_US * n {
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
