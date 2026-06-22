//! タイミング GNSS 受信機が報告する **量子化誤差 (qErr)** で PPS の測定を補正する。
//!
//! タイミング受信機 (u-blox NEO-M8T / ZED-F9T の UBX-TIM-TP、ST Teseo、Quectel L26-T 等) は、自分が
//! 出した 1PPS パルスが理想の UTC 秒からどれだけずれたか (`qErr`、符号付き、単位 ps) をパルス毎に報告する。
//! パルスは受信機内部クロックのグリッドにしか立てられず、理想秒はグリッドの隙間にあるので、この誤差は
//! ±(グリッド周期/2) の sawtooth になる。これを測定値から引けば sawtooth が消えて位相 σ が縮む。
//! MT3333 (現用) は qErr を出さないので、この補正は受信機を timing 系へ替えたときに効く一段である。
//!
//! 規約: **補正後のエッジ時刻 = 測定エッジ時刻 − qErr**。
//! 周波数推定が使う「連続エッジ間隔」は両エッジの補正を受けるので、間隔からは **qErr の差** を引く:
//! `interval_corr = (t2 − qerr2) − (t1 − qerr1) = interval_raw − (qerr2 − qerr1)`。
//! 位相 (offset②) は当該エッジの qErr をそのまま引く。
//! qErr の符号規約は受信機データシートに従い実機で検証する (ここは算術の building block で、呼び出し側が
//! 「corrected = captured − qerr」の符号で値を渡す)。
//!
//! 単位: qErr は ps、タイムスタンプ/間隔は ns。差を ps で取ってから ns へ丸める (各 qErr を先に ns 丸めして
//! から差を取るより精度が良い)。PIO 捕捉は 16ns 量子化だが、±~10ns の qErr を引くと逆に ~ns へ詰まる。

/// ps を ns へ四捨五入 (0 から離れる向き)。
const fn ps_to_ns_round(ps: i64) -> i64 {
    let bias = if ps >= 0 { 500 } else { -500 };
    (ps + bias) / 1000
}

/// PPS の生の位相/オフセット (ns) を qErr (ps) で補正する。補正後 = 生 − qErr。
pub fn correct_phase_ns(raw_phase_ns: i64, qerr_ps: i32) -> i64 {
    raw_phase_ns - ps_to_ns_round(qerr_ps as i64)
}

/// 連続 PPS エッジ間隔 (ns) を、前回と今回の qErr (ps) で補正する純粋関数。状態を持たない building block。
/// 両エッジが「補正後 = 測定 − qErr」を受けるので、間隔からは qErr の差 `now − prev` を引く。
/// 差は ps で取ってから ns へ丸める (各 qErr を先に丸めるより精度が良い)。i32 差を i64 で取り overflow を避ける。
pub fn correct_interval_ns(raw_interval_ns: i64, qerr_prev_ps: i32, qerr_now_ps: i32) -> i64 {
    raw_interval_ns - ps_to_ns_round(qerr_now_ps as i64 - qerr_prev_ps as i64)
}

/// 連続 PPS エッジ間隔 (ns) を qErr で補正する小さな状態 (直前エッジの qErr を保持する)。
///
/// `correct_interval_ns` は生間隔から「今回 − 前回」の qErr 差 (ps→ns 丸め) を引く。最初のエッジは前回が
/// 無いので生のまま返す (そもそも最初のエッジには間隔が無い)。状態は前回 qErr の一つだけ。
#[derive(Debug, Default, Clone, Copy)]
pub struct QErrCorrector {
    prev_qerr_ps: Option<i32>,
}

impl QErrCorrector {
    /// 直前 qErr 未設定の新しい corrector。
    pub const fn new() -> Self {
        Self { prev_qerr_ps: None }
    }

    /// 生間隔 (ns) を、前回エッジから今回エッジへの qErr 変化分で補正して返す。今回 qErr を内部に記録する。
    /// 最初のエッジ (前回 qErr 無し) は生間隔をそのまま返す。
    pub fn correct_interval_ns(&mut self, raw_interval_ns: i64, qerr_now_ps: i32) -> i64 {
        let corrected = match self.prev_qerr_ps {
            Some(prev) => correct_interval_ns(raw_interval_ns, prev, qerr_now_ps),
            None => raw_interval_ns,
        };
        self.prev_qerr_ps = Some(qerr_now_ps);
        corrected
    }

    /// 直前 qErr の記録を捨てる (PPS 欠落/再ロックで間隔の連続性が切れたとき。次のエッジは生間隔を返す)。
    pub fn reset(&mut self) {
        self.prev_qerr_ps = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phase_subtracts_qerr_with_rounding() {
        // 補正後 = 生 − qErr(ps→ns 丸め)。
        assert_eq!(correct_phase_ns(100, 4_000), 96); // 4000ps = 4ns
        assert_eq!(correct_phase_ns(100, -4_000), 104);
        assert_eq!(correct_phase_ns(0, 1_400), -1); // 1400ps → 1ns 丸め
        assert_eq!(correct_phase_ns(0, 1_600), -2); // 1600ps → 2ns 丸め
        assert_eq!(correct_phase_ns(0, -1_600), 2);
        assert_eq!(correct_phase_ns(50, 0), 50); // qErr=0 は不変
    }

    #[test]
    fn first_interval_is_unchanged() {
        let mut c = QErrCorrector::new();
        // 最初のエッジは前回 qErr が無いので生のまま (間隔自体まだ無いが、API 上は raw を返す)。
        assert_eq!(c.correct_interval_ns(1_000_000_000, 3_000), 1_000_000_000);
    }

    #[test]
    fn constant_qerr_leaves_interval_unchanged() {
        // qErr が一定なら差は 0 で間隔は不変 (定数オフセットは周波数に効かない)。
        let mut c = QErrCorrector::new();
        c.correct_interval_ns(1_000_000_000, 5_000); // prime prev
        assert_eq!(c.correct_interval_ns(1_000_000_000, 5_000), 1_000_000_000);
        assert_eq!(c.correct_interval_ns(1_000_000_000, 5_000), 1_000_000_000);
    }

    #[test]
    fn interval_subtracts_qerr_difference() {
        let mut c = QErrCorrector::new();
        c.correct_interval_ns(1_000_000_000, 2_000); // prev = 2000ps
        // 今回 qErr=7000ps、差 +5000ps = +5ns → 間隔から 5ns 引く。
        assert_eq!(c.correct_interval_ns(1_000_000_000, 7_000), 999_999_995);
        // 次は prev=7000、今回 2000、差 -5000ps = -5ns → 間隔に 5ns 足す。
        assert_eq!(c.correct_interval_ns(1_000_000_000, 2_000), 1_000_000_005);
    }

    #[test]
    fn sawtooth_qerr_is_removed_from_intervals() {
        // 受信機の sawtooth: 真の秒は一定 (間隔 1e9) だが、捕捉した生間隔は qErr 差ぶん揺れる。
        // 生間隔[n] = 1e9 + (qerr[n] − qerr[n-1])/1000。補正でこれを消すと全て 1e9 へ戻る。
        let qerr_ps = [0, 8_000, -8_000, 8_000, -8_000, 0]; // ±8ns sawtooth
        let mut c = QErrCorrector::new();
        let mut prev = qerr_ps[0];
        c.correct_interval_ns(1_000_000_000, qerr_ps[0]); // prime
        for &q in &qerr_ps[1..] {
            let raw = 1_000_000_000 + ps_to_ns_round(q as i64 - prev as i64);
            let corrected = c.correct_interval_ns(raw, q);
            assert_eq!(corrected, 1_000_000_000, "sawtooth not removed at qerr={q}");
            prev = q;
        }
    }

    #[test]
    fn pure_interval_matches_struct_and_handles_extremes() {
        // 純粋関数は (raw, prev, now) から直接補正。struct と同じ結果。
        assert_eq!(correct_interval_ns(1_000_000_000, 2_000, 7_000), 999_999_995);
        assert_eq!(correct_interval_ns(1_000_000_000, 7_000, 2_000), 1_000_000_005);
        assert_eq!(correct_interval_ns(1_000_000_000, 5_000, 5_000), 1_000_000_000);
        // i32 の両極端の差 (~±4.29e9 ps = ±4.29ms) を i64 で取り overflow しない。
        let big = correct_interval_ns(1_000_000_000, i32::MIN, i32::MAX);
        assert_eq!(big, 1_000_000_000 - ps_to_ns_round(i32::MAX as i64 - i32::MIN as i64));
        assert!(big < 1_000_000_000); // now>prev なので間隔は縮む
    }

    #[test]
    fn rounding_boundaries() {
        // 0 から離れる向きの四捨五入 (round-half-away-from-zero)。
        assert_eq!(correct_phase_ns(0, 500), -1); //  500ps → 1ns
        assert_eq!(correct_phase_ns(0, 499), 0); //  499ps → 0ns
        assert_eq!(correct_phase_ns(0, -500), 1); // -500ps → -1ns → 引いて +1
        assert_eq!(correct_phase_ns(0, -499), 0);
        assert_eq!(correct_interval_ns(1_000_000_000, 0, 1_500), 999_999_998); // 1500ps → 2ns
        assert_eq!(correct_interval_ns(1_000_000_000, 0, 2_500), 999_999_997); // 2500ps → 3ns
    }

    #[test]
    fn reset_drops_prev_so_next_interval_is_raw() {
        let mut c = QErrCorrector::new();
        c.correct_interval_ns(1_000_000_000, 9_000);
        c.reset();
        // reset 後の最初は生のまま (欠落跨ぎの間隔は qErr 差を当てない)。
        assert_eq!(c.correct_interval_ns(1_000_000_000, 1_000), 1_000_000_000);
        // 以降は通常どおり差で補正。
        assert_eq!(c.correct_interval_ns(1_000_000_000, 4_000), 999_999_997);
    }
}
