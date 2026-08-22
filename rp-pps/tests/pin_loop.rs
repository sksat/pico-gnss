//! 1PPS 出力を「ピンの実測」で閉じるループの、プラントモデルに対する性質試験。
//!
//! [`PpsSchedule`] が動かすのはピンの**位相**である。周期語は 1 回の間隔を決め、その間隔が次の
//! エッジの位置に足し込まれる。つまりプラントは積分器であり、そこに比例の補正を掛けると一次の
//! ループになる。積分器をもう 1 つ前置きすると二次になり、減衰項が無いので鳴る。
//!
//! 実機で鳴った。ピンと受信機の 1PPS の差の自己相関が符号を交互に振りながら減衰し (−0.82, +0.58,
//! −0.41, +0.30)、比例だけに直すと符号の交互振れが消えた (+0.34, +0.11)。この試験はその結論を
//! モデルの側に固定するもので、**実機の数値を通すためのものではない**。入力は一般的な物理レンジ
//! で与え、特定の基板の値はハードコードしない。
//!
//! プラントの式は
//!
//! ```text
//! ピンの位相(n) = schedule のエッジ(n) + 出力チェーンの遅れ − 基準の秒(n)
//! ```
//!
//! で、基準の秒は真の 1 秒ごとに `1e9 · (1 + δ)` 公称 ns 進む (δ は水晶の周波数誤差)。
//! これは定義的恒等であって、量子化・dither・比較器の遅れといった未モデルの I/O は保証しない。

use proptest::prelude::*;
use rp_pps::{PpsSchedule, PpsScheduleConfig, output_high_cycles, output_period_cycles};

const CLK: u32 = 125_000_000;

/// ピンの位相を追うプラント。
///
/// `chain_ns` は出力プログラムとパッドと入力捕捉の固定の遅れで、ループから見れば定数のオフセット
/// である。`rate_ppb` は水晶の周波数誤差。
struct Plant {
    schedule: PpsSchedule,
    high: u32,
    ref_ns: i128,
    chain_ns: i64,
    rate_ppb: i64,
}

impl Plant {
    fn new(rate_ppb: i64, chain_ns: i64, start_error_ns: i64) -> Self {
        let high = output_high_cycles(CLK, 100_000_000);
        let initial = output_period_cycles(CLK, high);
        let mut schedule =
            PpsSchedule::at_enable(CLK, high, PpsScheduleConfig::default(), 0, initial);
        // 最初のエッジをどこから始めるかが初期位相誤差になる。
        let _ = schedule.step(0, 0);
        // 基準の秒の原点をずらして、初期位相誤差を作る。
        let ref_ns = schedule.edge_ns() as i128 + chain_ns as i128 - start_error_ns as i128;
        Self {
            schedule,
            high,
            ref_ns,
            chain_ns,
            rate_ppb,
        }
    }

    /// 真の 1 秒ぶん、基準を進める。公称 ns での歩幅は水晶の誤差だけ長い (または短い)。
    fn advance_reference(&mut self) {
        self.ref_ns += 1_000_000_000 + (1_000_000_000i128 * self.rate_ppb as i128) / 1_000_000_000;
    }

    /// ピンが立った位置の、基準の秒からのずれ。出力チェーンの遅れが乗る。
    ///
    /// 秒の中に折り返す。エッジが 1 秒ずれても「同じ秒の 1 ns 手前」と区別がつかないのは、
    /// 実機で測っている量も同じである。
    fn pin_phase_ns(&self) -> i64 {
        fold(self.schedule.edge_ns() as i128 + self.chain_ns as i128 - self.ref_ns)
    }

    /// schedule 自身が思っているエッジの位置。map はチェーンの遅れを知らないので、そこは乗らない。
    fn map_phase_ns(&self) -> i64 {
        fold(self.schedule.edge_ns() as i128 - self.ref_ns)
    }
}

/// 秒の中への折り返し。firmware の `pps_lateness_ns` と同じ。
fn fold(v: i128) -> i64 {
    let into = v.rem_euclid(1_000_000_000);
    (if into > 500_000_000 {
        into - 1_000_000_000
    } else {
        into
    }) as i64
}

/// 測定の量子化。呼び出し側がレンジで与える (特定の HW の tick をここに書かない)。
fn quantise(v: i64, tick_ns: i64) -> i64 {
    if tick_ns <= 1 {
        return v;
    }
    (v as f64 / tick_ns as f64).round() as i64 * tick_ns
}

/// 比例だけのループ。プラントが積分器なので、これで一次になる。
fn run_proportional(
    rate_ppb: i64,
    chain_ns: i64,
    start_error_ns: i64,
    tick_ns: i64,
    steps: usize,
) -> Vec<i64> {
    let mut p = Plant::new(rate_ppb, chain_ns, start_error_ns);
    let mut out = Vec::with_capacity(steps);
    let mut measured = 0i64;
    for _ in 0..steps {
        // 前のエッジについて測った値で、次のエッジを置く。1 サンプルの遅れがある。
        p.schedule.advance(rate_ppb * 1_000, measured);
        p.advance_reference();
        measured = quantise(p.pin_phase_ns(), tick_ns);
        out.push(measured);
    }
    out
}

/// 積分器を 1 つ挟んだループ。実機で鳴っていた形。
fn run_with_integrator(
    rate_ppb: i64,
    chain_ns: i64,
    start_error_ns: i64,
    tick_ns: i64,
    steps: usize,
    gain_inv: i64,
) -> Vec<i64> {
    let mut p = Plant::new(rate_ppb, chain_ns, start_error_ns);
    let mut out = Vec::with_capacity(steps);
    let mut measured = 0i64;
    let mut integral = 0i64;
    for _ in 0..steps {
        // 補正は積分器に溜まり、schedule はその積分器を通した量を見る。schedule 自身の位相と
        // 積分器の和が 0 になるように動くので、状態が 2 つある。
        integral += measured / gain_inv;
        // schedule が見るのは map を通した自分の位相と、積分器の和である。map はチェーンの遅れを
        // 知らないので、積分器がそれを埋める役をしている — それが 2 つめの状態になる。
        let own = p.map_phase_ns();
        p.schedule.advance(rate_ppb * 1_000, own + integral);
        p.advance_reference();
        measured = quantise(p.pin_phase_ns(), tick_ns);
        out.push(measured);
    }
    out
}

/// schedule に自分の位相だけを見せるループ。ピンの遅れは誰も埋めない。
fn run_map_only(
    rate_ppb: i64,
    chain_ns: i64,
    start_error_ns: i64,
    tick_ns: i64,
    steps: usize,
) -> Vec<i64> {
    let mut p = Plant::new(rate_ppb, chain_ns, start_error_ns);
    let mut out = Vec::with_capacity(steps);
    for _ in 0..steps {
        let own = p.map_phase_ns();
        p.schedule.advance(rate_ppb * 1_000, own);
        p.advance_reference();
        out.push(quantise(p.pin_phase_ns(), tick_ns));
    }
    out
}

/// 符号が変わった回数。行き過ぎていれば何度も変わる。
fn sign_changes(v: &[i64]) -> usize {
    v.windows(2)
        .filter(|w| w[0] != 0 && w[1] != 0 && (w[0] > 0) != (w[1] > 0))
        .count()
}

/// lag 1 の自己相関を、`every` 個おきに間引いた列で見る。
///
/// 間引く理由がある。振動の周期がサンプルより十分長いと、隣り合うサンプルは同じ向きに動くので
/// 自己相関は正に出る。実機のログは 16 秒おきで、周期が 26 秒ほどだったので隣り合う記録が半周期
/// より少し先を向き、符号が交互になって見えた。同じ間引きをすれば同じ形が出る。
fn acf1_every(v: &[i64], every: usize) -> f64 {
    let d: Vec<i64> = v.iter().step_by(every).copied().collect();
    let n = d.len();
    if n < 3 {
        return 0.0;
    }
    let mean = d.iter().sum::<i64>() as f64 / n as f64;
    let num: f64 = (1..n)
        .map(|i| (d[i] as f64 - mean) * (d[i - 1] as f64 - mean))
        .sum();
    let den: f64 = d.iter().map(|&x| (x as f64 - mean).powi(2)).sum();
    if den == 0.0 { 0.0 } else { num / den }
}

/// 一周期あたりの公称サイクル。位相をこれより細かくは置けない。
const CYCLE_NS: i64 = 1_000_000_000 / CLK as i64;

#[test]
fn steering_by_the_pin_needs_no_integrator_and_does_not_ring() {
    // ピンの誤差をそのまま比例で返す。次の誤差は (1 - g) 倍で、状態は増えない。
    let e = run_proportional(2_000, 400, 1_000, 0, 60);
    assert_eq!(sign_changes(&e), 0, "行き過ぎない: {:?}", &e[..12]);
    assert!(
        e.last().unwrap().abs() <= 2 * CYCLE_NS,
        "0 に落ちる。出力チェーンの遅れは残らない: 最後は {} ns",
        e.last().unwrap()
    );
    for w in e[..20].windows(2) {
        assert!(w[1].abs() <= w[0].abs(), "単調に減る: {:?}", &e[..20]);
    }
}

#[test]
fn steering_by_the_map_alone_leaves_the_output_chain_standing() {
    // schedule が見るのが自分の位相だけだと、ピンとのあいだの固定の遅れは誰も埋めない。
    // これが積分器を足した理由である。
    let chain = 400;
    let e = run_map_only(2_000, chain, 1_000, 0, 60);
    assert_eq!(sign_changes(&e), 0, "鳴りはしない");
    assert!(
        (e.last().unwrap() - chain).abs() <= 2 * CYCLE_NS,
        "チェーンのぶんが残る: 最後は {} ns、チェーンは {} ns",
        e.last().unwrap(),
        chain
    );
}

/// ループの形だけを取り出した線形モデル。
///
/// `PpsSchedule` の量子化・端数のキャリー・クランプを通さない。鳴るかどうかは、状態がいくつ
/// あって、それぞれがどの順で更新されるかで決まる — つまりループの形の性質であって、周期語を
/// どう丸めるかの性質ではない。ここではその形だけを見る。
///
/// `own` は schedule が思っている位相、`integral` は補正の溜まり、`chain` は出力チェーンの遅れ。
fn topology(
    with_integrator: bool,
    gain_inv: f64,
    chain: f64,
    start: f64,
    steps: usize,
) -> Vec<f64> {
    let (mut own, mut integral) = (start, 0.0);
    let mut out = Vec::with_capacity(steps);
    for _ in 0..steps {
        // schedule が見る量。積分器があれば自分の位相と溜まりの和、無ければピンの誤差そのもの。
        let seen = if with_integrator {
            own + integral
        } else {
            own + chain
        };
        own -= seen / gain_inv;
        let pin = own + chain;
        if with_integrator {
            integral += pin / gain_inv;
        }
        out.push(pin);
    }
    out
}

fn sign_changes_f(v: &[f64]) -> usize {
    v.windows(2)
        .filter(|w| w[0] != 0.0 && w[1] != 0.0 && (w[0] > 0.0) != (w[1] > 0.0))
        .count()
}

#[test]
fn the_two_topologies_differ_in_order_not_in_tuning() {
    // 同じゲイン、同じチェーン、同じ初期位相。違うのは積分器があるかどうかだけ。
    let with = topology(true, 4.0, 400.0, 1_000.0, 200);
    let without = topology(false, 4.0, 400.0, 1_000.0, 200);

    // どちらも 0 に落ちる。積分器はチェーンを埋めるためにあり、そこは果たす。
    assert!(with.last().unwrap().abs() < 1.0, "{}", with.last().unwrap());
    assert!(
        without.last().unwrap().abs() < 1.0,
        "{}",
        without.last().unwrap()
    );

    // 落ち方が違う。積分器のあるほうは行き過ぎ、無いほうは一度も跨がない。
    assert!(
        sign_changes_f(&with) >= 3,
        "積分器があると何度も行き過ぎる: {} 回",
        sign_changes_f(&with)
    );
    assert_eq!(
        sign_changes_f(&without),
        0,
        "積分器が無ければ跨がない: {} 回",
        sign_changes_f(&without)
    );
}

#[test]
fn the_ring_has_the_period_the_eigenvalues_say() {
    // 状態遷移は [[1 - g, -g], [g, 1]] で、g = 1/4 なら固有値は 0.875 +- 0.2165i。
    // |λ| = 0.901、偏角 0.2429 rad なので、周期は 2π/0.2429 = 25.9 サンプルになる。
    let v = topology(true, 4.0, 400.0, 1_000.0, 400);
    // ゼロ交差の間隔から周期を測る。半周期ごとに 1 回跨ぐ。
    let crossings: Vec<usize> = v
        .windows(2)
        .enumerate()
        .filter(|(_, w)| w[0] != 0.0 && w[1] != 0.0 && (w[0] > 0.0) != (w[1] > 0.0))
        .map(|(i, _)| i)
        .collect();
    assert!(crossings.len() >= 4, "交差が足りない: {crossings:?}");
    let half: Vec<usize> = crossings.windows(2).map(|w| w[1] - w[0]).collect();
    let mean_half = half.iter().sum::<usize>() as f64 / half.len() as f64;
    let period = 2.0 * mean_half;
    assert!(
        (period - 25.9).abs() < 3.0,
        "周期は 25.9 サンプルのはず: 実測 {period:.1}  半周期 {half:?}"
    );
}

#[test]
fn the_ringing_is_structural_not_a_matter_of_resolution() {
    // 測定の刻みを変えても、比例のループは刻みの上で震えるだけで行き過ぎない。
    for tick in [8, 16, 32] {
        let p = run_proportional(2_000, 400, 1_000, tick, 120);
        assert!(
            sign_changes(&p) <= 12,
            "tick {tick}: 刻みの上で震えるだけ ({} 回)",
            sign_changes(&p)
        );
        assert!(
            p.last().unwrap().abs() <= 2 * tick,
            "tick {tick}: 刻みまでは落ちる ({} ns)",
            p.last().unwrap()
        );
    }
}

/// ピンの誤差に対する PI。比例は位相を一度動かし、積分は周期を持ち続ける。
///
/// `rate_error` はこのループの外から周期に乗る誤差 (周波数推定が外している量) で、単位は 1 秒
/// あたりの ns。比例だけのループはこれに対して定常誤差を持ち、積分はそれを消す。
fn pi_loop(kp: f64, ki: f64, chain: f64, start: f64, rate_error: f64, steps: usize) -> Vec<f64> {
    let (mut own, mut integral) = (start, 0.0);
    let mut out = Vec::with_capacity(steps);
    let mut e = own + chain;
    for _ in 0..steps {
        // 前のエッジで測った誤差で次を置く。比例は位相へ、積分は周期へ。
        own += rate_error - kp * e - integral;
        integral += ki * e;
        e = own + chain;
        out.push(e);
    }
    out
}

#[test]
fn critical_damping_fixes_the_integral_gain_from_the_proportional_one() {
    // 離散の特性方程式は z^2 - (2 - Kp) z + (1 - Kp + Ki) で、重根を持つ条件は
    // (2 - Kp)^2 = 4 (1 - Kp + Ki)、つまり Ki = Kp^2 / 4 である。
    // 選ぶ数ではなく、Kp から決まる関係である。
    let kp = 0.25;
    let ki = kp * kp / 4.0;

    // 周期に 40 ns/s の誤差を乗せる。周波数推定がその場で外している量にあたる。
    let rate_error = 40.0;
    let p_only = pi_loop(kp, 0.0, 400.0, 0.0, rate_error, 400);
    let pi = pi_loop(kp, ki, 400.0, 0.0, rate_error, 400);

    let p_final = p_only.last().unwrap();
    assert!(
        (p_final.abs() - rate_error / kp).abs() < 1.0,
        "比例だけなら定常誤差は rate/Kp = {} ns: 実測 {p_final:.1}",
        rate_error / kp
    );
    assert!(
        pi.last().unwrap().abs() < 1.0,
        "PI なら定常誤差は消える: {:.1}",
        pi.last().unwrap()
    );

    let tail = &pi[20..];
    assert!(
        sign_changes_f(tail) <= 1,
        "臨界減衰なので跨ぐのは一度まで: {} 回",
        sign_changes_f(tail)
    );
}

#[test]
fn too_much_integral_gain_is_what_rang() {
    // 元の firmware は積分にも比例と同じ 1/4 を掛けていた。臨界減衰の Kp^2/4 に対して 16 倍である。
    let kp = 0.25;
    let critical = kp * kp / 4.0;
    let as_shipped = kp;
    assert!((as_shipped / critical - 16.0f64).abs() < 0.01, "16 倍");

    let rang = pi_loop(kp, as_shipped, 400.0, 1_000.0, 0.0, 400);
    let damped = pi_loop(kp, critical, 400.0, 1_000.0, 0.0, 400);
    assert!(
        sign_changes_f(&rang) >= 3,
        "16 倍だと何度も行き過ぎる: {} 回",
        sign_changes_f(&rang)
    );
    assert!(
        sign_changes_f(&damped) <= 1,
        "臨界減衰なら跨ぐのは一度まで: {} 回",
        sign_changes_f(&damped)
    );
}

proptest! {
    /// 一般的な物理レンジで、ピンで閉じた比例のループは落ち着く。
    ///
    /// 水晶は ±100 ppm、初期位相は ±0.5 秒、出力チェーンは ±10 µs、測定の刻みは 1〜64 ns。
    /// どれも特定の基板の値ではない。
    #[test]
    fn the_proportional_loop_settles_over_the_physical_range(
        rate_ppb in -100_000i64..100_000,
        start_error_ns in -500_000_000i64..500_000_000,
        chain_ns in -10_000i64..10_000,
        tick_ns in 1i64..64,
    ) {
        let e = run_proportional(rate_ppb, chain_ns, start_error_ns, tick_ns, 400);
        let tail = &e[e.len() - 40..];
        let worst = tail.iter().map(|x| x.abs()).max().unwrap();
        // 落ち着き先は、測定の刻みか周期語の刻みか、粗いほうで決まる。
        prop_assert!(
            worst <= 4 * tick_ns.max(CYCLE_NS),
            "rate {rate_ppb} start {start_error_ns} tick {tick_ns}: 最後の 40 秒で最大 {worst} ns"
        );
    }
}
