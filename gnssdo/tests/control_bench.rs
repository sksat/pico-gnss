//! Host plant-model comparison of the selectable [`PhaseController`]s — the mechanism for verifying
//! control methods on a model (the user-requested "plant model で host 検証できる仕組み").
//!
//! It mirrors `report/ctrl_eval.py`'s plant so the **Rust library is the single source of truth**:
//! each method is exercised through the same `PhaseController` trait, so a difference in the table is
//! a difference in the *method*, not in a re-implementation. Two measurement regimes are modelled —
//! `pio` (≈16 ns hardware capture) and `no-pio` (≈µs-scale GPIO capture + jitter) — so the historical
//! loopback/PIO-less configurations are comparable too.
//!
//! IMPORTANT (the measurement trap, see `report/REPORT.md`): the model is crude and, on hardware, the
//! loop measures output-vs-**the same receiver**, so a steady-state improvement here is NOT proof of
//! a true-UTC improvement. This harness ranks *dynamics* (lock, reacquire, step, drift) and flags
//! gross regressions; the reception-independent go/no-go is the hardware PRBS `h[k]` (separate).
//!
//! Run the comparison table with:
//!   cargo test -p gnssdo --test control_bench -- --ignored compare_controllers --nocapture

use gnssdo::{
    AlphaBetaBoost, ControlInput, IntegralRework, OpenLoopFf, PhaseController, PhaseLockLoop,
    PhaseLockLoopConfig,
};

// --- deterministic RNG (xorshift64*) so runs are reproducible without an external crate ---
struct Rng(u64);
impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed ^ 0x9E37_79B9_7F4A_7C15)
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Standard-normal via Box–Muller.
    fn gauss(&mut self) -> f64 {
        let u1 = self.unit().max(1e-12);
        let u2 = self.unit();
        (-2.0 * u1.ln()).sqrt() * (core::f64::consts::TAU * u2).cos()
    }
}

// Colored reception noise = random-walk + white, matching ctrl_eval.py's σ's.
const SIGMA_RW: f64 = 3.0;
const SIGMA_W: f64 = 12.0;

/// Reception-phase generator with explicit random-walk / white σ (ns). The random-walk gives the
/// low-frequency receiver-PPS wander that, passed through the underdamped loop sensitivity, becomes
/// the dominant output wander on hardware (see `calibrate_reception_to_real_spectrum`).
fn gen_reception_rw(n: usize, seed: u64, sigma_rw: f64, sigma_w: f64) -> Vec<f64> {
    let mut r = Rng::new(seed);
    let mut walk = 0.0;
    (0..n)
        .map(|_| {
            walk += sigma_rw * r.gauss();
            walk + sigma_w * r.gauss()
        })
        .collect()
}

fn gen_reception(n: usize, seed: u64) -> Vec<f64> {
    gen_reception_rw(n, seed, SIGMA_RW, SIGMA_W)
}

/// Realistic reception: a **smoothed** (1-pole low-pass, time constant `tau_lp` s) random-walk for the
/// slow receiver-PPS wander + per-edge white jitter. Smoothing keeps the low-frequency wander (which
/// the underdamped loop turns into the dominant output excursion) while removing the per-step
/// jumpiness of a raw random-walk, so the closed-loop output matches the *measured* hardware spectrum
/// (σ~200 ns dominated by 64-300 s, adjacent-edge jitter ~16 ns ≪ σ). See
/// `calibrate_reception_to_real_spectrum`.
fn gen_reception_smooth(n: usize, seed: u64, sigma_rw: f64, tau_lp: f64, sigma_w: f64) -> Vec<f64> {
    let mut r = Rng::new(seed);
    let (mut walk, mut smooth) = (0.0, 0.0);
    (0..n)
        .map(|_| {
            walk += sigma_rw * r.gauss();
            smooth += (walk - smooth) / tau_lp;
            smooth + sigma_w * r.gauss()
        })
        .collect()
}

/// A phase-measurement regime: capture quantization and added jitter (ns).
#[derive(Clone, Copy)]
struct Regime {
    name: &'static str,
    quant_ns: f64,
    jitter_ns: f64,
}
/// PIO hardware capture: ~16 ns quantization, negligible extra jitter.
const PIO: Regime = Regime {
    name: "pio",
    quant_ns: 16.0,
    jitter_ns: 0.0,
};
/// Loopback/PIO-less capture: ~µs GPIO-interrupt quantization and ~µs jitter (the regime PIO was
/// introduced to escape; see report's "PIO ハードキャプチャ" and "ループバック無しの構成").
const NO_PIO: Regime = Regime {
    name: "no-pio",
    quant_ns: 1_000.0,
    jitter_ns: 1_000.0,
};

fn quantize(x: f64, q: f64) -> i64 {
    ((x / q).round() * q) as i64
}

#[allow(clippy::too_many_arguments)]
fn run_plant(
    c: &mut dyn PhaseController,
    n: usize,
    rec: &[f64],
    reg: Regime,
    drift_accel: f64,
    step_at: Option<usize>,
    step_amp: f64,
    init_off: f64,
    seed: u64,
) -> (Vec<f64>, Vec<bool>) {
    let mut out = init_off;
    let (mut drift, mut rate) = (0.0, 0.0);
    let mut hw = vec![0.0; n];
    let mut lk = vec![false; n];
    let mut jr = Rng::new(seed ^ 0xABCD);
    for k in 0..n {
        rate += drift_accel;
        drift += rate;
        if Some(k) == step_at {
            out += step_amp;
        }
        let jitter = if reg.jitter_ns > 0.0 {
            reg.jitter_ns * jr.gauss()
        } else {
            0.0
        };
        let err = out - (rec[k] + drift) + jitter;
        let eq = quantize(err, reg.quant_ns);
        let o = c.step(ControlInput {
            err_ns: eq,
            valid: true,
        });
        hw[k] = err;
        lk[k] = c.is_locked();
        out += o.trim_mppb as f64 / 1000.0 - o.pcorr_ns as f64;
    }
    (hw, lk)
}

/// Like [`run_plant`] but with an extra **actuation delay** of `delay` edges: the controller's effect
/// on the output (trim + pcorr) reaches the plant `delay` edges after it is computed, modelling the
/// real output pipeline (period-word FIFO + freshness gating) that the 1-edge Smith predictor
/// under-compensates on hardware. `delay`=1 reproduces [`run_plant`] exactly (the model's implicit
/// 1-edge delay). Used to test whether matching `smith_edges` to the true delay restores damping.
#[allow(clippy::too_many_arguments)]
fn run_plant_delayed(
    c: &mut dyn PhaseController,
    n: usize,
    rec: &[f64],
    reg: Regime,
    drift_accel: f64,
    init_off: f64,
    seed: u64,
    delay: usize,
) -> (Vec<f64>, Vec<bool>) {
    let mut out = init_off;
    let (mut drift, mut rate) = (0.0, 0.0);
    let mut hw = vec![0.0; n];
    let mut lk = vec![false; n];
    let mut jr = Rng::new(seed ^ 0xABCD);
    let mut pipe = vec![0.0; delay.max(1)]; // effects scheduled `delay` edges ahead (ring)
    for k in 0..n {
        rate += drift_accel;
        drift += rate;
        let jitter = if reg.jitter_ns > 0.0 {
            reg.jitter_ns * jr.gauss()
        } else {
            0.0
        };
        let err = out - (rec[k] + drift) + jitter;
        let eq = quantize(err, reg.quant_ns);
        let o = c.step(ControlInput {
            err_ns: eq,
            valid: true,
        });
        hw[k] = err;
        lk[k] = c.is_locked();
        // apply the effect scheduled for now (computed `delay` edges ago); schedule this edge's effect.
        let slot = k % pipe.len();
        out += pipe[slot];
        pipe[slot] = o.trim_mppb as f64 / 1000.0 - o.pcorr_ns as f64;
    }
    (hw, lk)
}

fn std_dev(xs: &[f64]) -> f64 {
    let m = xs.iter().sum::<f64>() / xs.len() as f64;
    (xs.iter().map(|x| (x - m).powi(2)).sum::<f64>() / xs.len() as f64).sqrt()
}

#[derive(Default, Clone)]
struct Metrics {
    steady_rms: f64,
    reacq_1us_edge: f64,
    lock_edge: f64,
    step_settle_edge: f64,
    step_overshoot_ns: f64,
    step_zerocross: f64,
    drift_rms: f64,
}

type Factory = fn() -> Box<dyn PhaseController>;

fn evaluate(make: Factory, reg: Regime, seeds: u64) -> Metrics {
    let mut m = Metrics::default();
    for s in 0..seeds {
        // Each sub-experiment draws BOTH its reception (gen_reception offset) and its plant jitter
        // (run_plant seed) from a distinct stream, so the four no-PIO traces are independent for each
        // s — reusing one jitter seed `s` across all four would correlate them and shrink the
        // effective sample count behind each averaged metric.
        // steady
        let rec = gen_reception(6000, 1000 + s);
        let (hw, _) = run_plant(&mut *make(), 6000, &rec, reg, 0.0, None, 0.0, 0.0, 5000 + s);
        m.steady_rms += std_dev(&hw[1500..]);
        // reacquire from 50 µs
        let rec = gen_reception(1500, 2000 + s);
        let (hw, lk) = run_plant(&mut *make(), 1500, &rec, reg, 0.0, None, 0.0, 50_000.0, 6000 + s);
        m.reacq_1us_edge += hw.iter().position(|v| v.abs() < 1000.0).unwrap_or(1500) as f64;
        m.lock_edge += lk.iter().position(|&v| v).unwrap_or(1500) as f64;
        // step
        let rec = gen_reception(2000, 3000 + s);
        let (hw, _) = run_plant(&mut *make(), 2000, &rec, reg, 0.0, Some(800), 2000.0, 0.0, 7000 + s);
        let seg = &hw[800..1600];
        m.step_settle_edge += seg.iter().position(|v| v.abs() < 200.0).unwrap_or(800) as f64;
        m.step_overshoot_ns += seg.iter().fold(0.0_f64, |a, v| a.max(v.abs()));
        m.step_zerocross += seg
            .windows(2)
            .filter(|w| (w[0] > 0.0) != (w[1] > 0.0))
            .count() as f64;
        // drift
        let rec = gen_reception(3000, 4000 + s);
        let (hw, _) = run_plant(&mut *make(), 3000, &rec, reg, 0.002, None, 0.0, 0.0, 8000 + s);
        m.drift_rms += std_dev(&hw[1500..]);
    }
    let n = seeds as f64;
    Metrics {
        steady_rms: m.steady_rms / n,
        reacq_1us_edge: m.reacq_1us_edge / n,
        lock_edge: m.lock_edge / n,
        step_settle_edge: m.step_settle_edge / n,
        step_overshoot_ns: m.step_overshoot_ns / n,
        step_zerocross: m.step_zerocross / n,
        drift_rms: m.drift_rms / n,
    }
}

// --- the selectable controller zoo (every historical + candidate method) ---
fn zoo() -> Vec<(&'static str, Factory)> {
    vec![
        ("open_loop", || Box::new(OpenLoopFf::new())),
        ("naive_pid", || {
            Box::new(PhaseLockLoop::with_config(PhaseLockLoopConfig::NAIVE_PID))
        }),
        ("pll_smith_128", || Box::new(PhaseLockLoop::new())),
        ("pll_smith_512", || {
            Box::new(PhaseLockLoop::with_config(PhaseLockLoopConfig {
                i_den: 512,
                ..PhaseLockLoopConfig::DEFAULT
            }))
        }),
        ("integ_rework", || Box::new(IntegralRework::new())),
        ("ab_boost", || Box::new(AlphaBetaBoost::new())),
    ]
}

#[test]
#[ignore = "comparison table; run manually with --ignored --nocapture"]
fn compare_controllers() {
    for reg in [PIO, NO_PIO] {
        println!(
            "\n# regime={} (quant={:.0}ns jitter={:.0}ns) — smaller is better; open_loop has no phase servo",
            reg.name, reg.quant_ns, reg.jitter_ns
        );
        println!(
            "{:>14} {:>10} {:>9} {:>8} {:>11} {:>9} {:>9} {:>9}",
            "controller",
            "steady_rms",
            "reacq1us",
            "lock",
            "step_settle",
            "overshoot",
            "step_zc",
            "drift_rms"
        );
        for (name, make) in zoo() {
            let m = evaluate(make, reg, 6);
            println!(
                "{:>14} {:>10.1} {:>9.1} {:>8.1} {:>11.1} {:>9.0} {:>9.1} {:>9.1}",
                name,
                m.steady_rms,
                m.reacq_1us_edge,
                m.lock_edge,
                m.step_settle_edge,
                m.step_overshoot_ns,
                m.step_zerocross,
                m.drift_rms
            );
        }
    }
}

// --- regression guards (cheap, always run) ---

#[test]
fn every_closed_loop_controller_locks_under_pio() {
    // From a 50 µs offset, every phase-servo method must reach lock and settle inside the lock
    // window within the reacquire window. Open-loop is excluded (it has no phase feedback).
    for (name, make) in zoo() {
        if name == "open_loop" {
            continue;
        }
        let rec = gen_reception(1500, 42);
        let (hw, lk) = run_plant(&mut *make(), 1500, &rec, PIO, 0.0, None, 0.0, 50_000.0, 7);
        assert!(lk.iter().any(|&v| v), "{name} never locked from 50µs");
        let tail = std_dev(&hw[1000..]);
        assert!(tail < 1000.0, "{name} did not settle (tail σ={tail:.0}ns)");
    }
}

#[test]
fn open_loop_never_servos_phase() {
    let rec = gen_reception(500, 9);
    let mut c = OpenLoopFf::new();
    let (_hw, lk) = run_plant(&mut c, 500, &rec, PIO, 0.0, None, 0.0, 20_000.0, 3);
    assert!(!lk.iter().any(|&v| v), "open-loop must never claim lock");
}

#[test]
fn ab_boost_does_not_lag_badly_under_harsh_drift() {
    // Regression for the verification workflow's main robustness finding: the OLD open-loop predict
    // (phase_hat + freq/1000, ignoring the applied trim) gave AlphaBetaBoost a large steady-state lag
    // under a sustained ramp (~2300 ns at accel 0.05, vs the PI loops' ~7 ns). The closed-loop predict
    // makes its drift integrator a true type-II on the ramp. Under a harsh ramp, ab_boost's drift tail
    // must stay within a small multiple of pll_smith_128's — not the ~300× the defect produced.
    let harsh = 0.02;
    let drift_tail = |make: Factory| {
        let mut acc = 0.0;
        for s in 0..6u64 {
            let rec = gen_reception(3000, 4000 + s);
            let (hw, _) = run_plant(&mut *make(), 3000, &rec, PIO, harsh, None, 0.0, 0.0, 9000 + s);
            acc += std_dev(&hw[1500..]);
        }
        acc / 6.0
    };
    let ab = drift_tail(|| Box::new(AlphaBetaBoost::new()));
    let pi = drift_tail(|| Box::new(PhaseLockLoop::new()));
    assert!(
        ab < pi * 4.0 + 50.0,
        "ab_boost harsh-drift tail {ab:.0}ns should be within ~4x of pll_smith_128 {pi:.0}ns (was ~300x before the closed-loop-predict fix)"
    );
}

#[test]
fn ab_boost_reacquires_no_slower_than_naive_pid() {
    // The whole point of the transient boost is fast recovery. Average reacquire edge over seeds must
    // be no worse than the naive PID's. (A robust ordering check, not a fragile absolute target — we
    // do not over-fit the crude model; hardware PRBS decides adoption.)
    let ab = evaluate(|| Box::new(AlphaBetaBoost::new()), PIO, 6).reacq_1us_edge;
    let naive = evaluate(
        || Box::new(PhaseLockLoop::with_config(PhaseLockLoopConfig::NAIVE_PID)),
        PIO,
        6,
    )
    .reacq_1us_edge;
    assert!(
        ab <= naive + 1.0,
        "ab_boost reacq {ab:.1} should be ≤ naive_pid {naive:.1}"
    );
}

/// 自己相関の初期減衰後の最初のピークのラグ (= 卓越周期, サンプル≈秒)。
fn dominant_period(x: &[f64]) -> usize {
    let n = x.len();
    let m = x.iter().sum::<f64>() / n as f64;
    let var: f64 = x.iter().map(|v| (v - m).powi(2)).sum();
    if var <= 0.0 {
        return 0;
    }
    let ac = |lag: usize| -> f64 {
        x[..n - lag]
            .iter()
            .zip(&x[lag..])
            .map(|(a, b)| (a - m) * (b - m))
            .sum::<f64>()
            / var
    };
    let (mut prev, mut best) = (1.0, 0usize);
    for lag in 5..(n / 3).min(500) {
        let cur = ac(lag);
        let nxt = ac(lag + 1);
        if cur > prev && cur > nxt && cur > 0.1 {
            best = lag;
            break;
        }
        prev = cur;
    }
    best
}

/// **実機スペクトルへのプラント校正** (task #5: 実機がモデルから乖離したらモデルの仮説/実装を直す)。
///
/// 実機 hwphase は σ~200-220ns で電力の ~85% が 64-300s 帯 (ループ underdamped 共振 ~190s @ i_den=512)
/// に集中し、隣接ジッタ (<16s) は ~0% (= 低周波支配)。既定 SIGMA_RW=3 のモデルは閉ループ σ~16ns しか
/// 出さず実機と桁違い ― 受信機の低周波 PPS wander (random-walk 励起) が足りないのが原因。SIGMA_RW を掃引し、
/// production PLL (i_den=512/d16) の閉ループ σ・隣接ジッタ比・卓越周期が実機に合う点を探す。校正値は別途
/// 回帰テストに固定する。
///   cargo test -p gnssdo --test control_bench -- --ignored calibrate_reception_to_real_spectrum --nocapture
#[test]
#[ignore = "model-vs-hardware calibration sweep (prints a table)"]
fn calibrate_reception_to_real_spectrum() {
    let prod = || {
        Box::new(PhaseLockLoop::with_config(PhaseLockLoopConfig {
            i_den: 512,
            d_den: 16,
            ..PhaseLockLoopConfig::DEFAULT
        })) as Box<dyn PhaseController>
    };
    let (n, warm) = (4000usize, 400usize);
    eprintln!(
        "CALIBRATE production PLL (i_den=512/d16), n={n}; REAL target: sd~200-220ns, adj/sd<<1, period~190s"
    );
    eprintln!("  SIGMA_RW   sd[ns]  adjdiff[ns]  adj/sd  period[s]");
    for &rw in &[3.0, 10.0, 30.0, 60.0, 100.0, 150.0, 250.0] {
        let (mut sd_s, mut adj_s, mut per_s) = (0.0, 0.0, 0.0);
        let seeds = 4u64;
        for seed in 0..seeds {
            let rec = gen_reception_rw(n, 100 + seed, rw, SIGMA_W);
            let (hw, _) = run_plant(&mut *prod(), n, &rec, PIO, 0.0, None, 0.0, 0.0, 7000 + seed);
            let h = &hw[warm..];
            let adj: Vec<f64> = h.windows(2).map(|w| w[1] - w[0]).collect();
            sd_s += std_dev(h);
            adj_s += std_dev(&adj);
            per_s += dominant_period(h) as f64;
        }
        let (sd, adj, per) = (sd_s / seeds as f64, adj_s / seeds as f64, per_s / seeds as f64);
        eprintln!(
            "  {:7.0}   {:6.0}   {:8.0}    {:5.2}   {:7.0}",
            rw,
            sd,
            adj,
            adj / sd.max(1.0),
            per
        );
    }
    // 平滑版 (LPF'd random-walk + white): 形 (adj/sd~0.07) まで実機に合わせる掃引。
    eprintln!("\n  -- smoothed walk (tau_lp,sigma_rw) + white {SIGMA_W} --");
    eprintln!("  tau_lp sigma_rw   sd[ns]  adjdiff[ns]  adj/sd  period[s]");
    for &(tau, rw) in &[
        (8.0, 60.0),
        (8.0, 100.0),
        (8.0, 150.0),
        (15.0, 150.0),
        (15.0, 250.0),
        (30.0, 300.0),
    ] {
        let (mut sd_s, mut adj_s, mut per_s) = (0.0, 0.0, 0.0);
        let seeds = 4u64;
        for seed in 0..seeds {
            let rec = gen_reception_smooth(n, 100 + seed, rw, tau, SIGMA_W);
            let (hw, _) = run_plant(&mut *prod(), n, &rec, PIO, 0.0, None, 0.0, 0.0, 7000 + seed);
            let h = &hw[warm..];
            let adj: Vec<f64> = h.windows(2).map(|w| w[1] - w[0]).collect();
            sd_s += std_dev(h);
            adj_s += std_dev(&adj);
            per_s += dominant_period(h) as f64;
        }
        let (sd, adj, per) = (sd_s / seeds as f64, adj_s / seeds as f64, per_s / seeds as f64);
        eprintln!(
            "  {:6.0} {:8.0}   {:6.0}   {:8.0}    {:5.2}   {:7.0}",
            tau,
            rw,
            sd,
            adj,
            adj / sd.max(1.0),
            per
        );
    }
}

/// 校正された励起 = 平滑 random-walk (tau_lp=8, sigma_rw=150) + 白色 12ns。
/// 既定 `gen_reception` (SIGMA_RW=3) は閉ループ σ~16ns で実機 (~220ns) と桁違いだったので、実機 hwphase の
/// スペクトル (σ~220ns, 64-300s 帯支配, 隣接ジッタ~16ns ≪ σ) を再現する受信機励起を別に用意した。
const REAL_SIGMA_RW: f64 = 150.0;
const REAL_TAU_LP: f64 = 8.0;

/// 受信機励起を実機に合わせた版の生成 (controller を現実的プラントで比較したいとき用)。
fn gen_reception_real(n: usize, seed: u64) -> Vec<f64> {
    gen_reception_smooth(n, seed, REAL_SIGMA_RW, REAL_TAU_LP, SIGMA_W)
}

/// **プラントモデル ↔ 実機 スペクトル一致の回帰** (task #5)。校正した励起 `gen_reception_real` を
/// production PLL (i_den=512/d16) に通すと、閉ループ hwphase が実機の特徴を再現する:
/// σ~220ns (実機 223)、隣接ジッタ ≪ σ (低周波支配; adj/sd<0.30 vs 実機 0.07)、卓越周期は数百 s
/// (ループ underdamped 共振 2π√i_den の帯)。**board 固有の数値**なので #[ignore] (thermal_plant の
/// replay と同じ方針)。既定 `gen_reception` (σ~16ns) との乖離が「モデルは crude」の正体で、これを直すと
/// controller 比較を現実的な低周波 wander 下で行える (絶対 σ は依然 model 依存・実機 PRBS が go/no-go)。
///   cargo test -p gnssdo --test control_bench -- --ignored model_matches_real_wander_spectrum --nocapture
#[test]
#[ignore = "board-specific spectrum match (real ~220ns); run manually"]
fn model_matches_real_wander_spectrum() {
    let prod = || {
        Box::new(PhaseLockLoop::with_config(PhaseLockLoopConfig {
            i_den: 512,
            d_den: 16,
            ..PhaseLockLoopConfig::DEFAULT
        })) as Box<dyn PhaseController>
    };
    let (n, warm) = (4000usize, 400usize);
    let (mut sd_s, mut adj_s, mut per_s) = (0.0, 0.0, 0.0);
    let seeds = 6u64;
    for seed in 0..seeds {
        let rec = gen_reception_real(n, 200 + seed);
        let (hw, _) = run_plant(&mut *prod(), n, &rec, PIO, 0.0, None, 0.0, 0.0, 9000 + seed);
        let h = &hw[warm..];
        let adj: Vec<f64> = h.windows(2).map(|w| w[1] - w[0]).collect();
        sd_s += std_dev(h);
        adj_s += std_dev(&adj);
        per_s += dominant_period(h) as f64;
    }
    let (sd, adj, per) = (sd_s / seeds as f64, adj_s / seeds as f64, per_s / seeds as f64);
    eprintln!(
        "MODEL vs REAL: sd={sd:.0}ns (real 223) adj/sd={:.2} (real 0.07, low-freq) period={per:.0}s (real ~190, loop mode)",
        adj / sd
    );
    // 実機の主要な特徴を再現していること (board 固有なので幅は広め)。
    assert!(
        (150.0..320.0).contains(&sd),
        "model wander σ should match hardware ~220ns: {sd:.0}ns"
    );
    assert!(
        adj / sd < 0.30,
        "model wander must be low-frequency dominated like hardware (adj/sd≪1): {:.2}",
        adj / sd
    );
    assert!(
        (120.0..450.0).contains(&per),
        "dominant period should be in the loop-resonance band (hundreds of s): {per:.0}s"
    );
}

/// **遅延補償で減衰が効くか (本丸 #7 の host 検証)**。実機の wander はループ underdamped 共振で、減衰を
/// 上げる (kp_inv 8→4) と発振する — Smith が 1 エッジしか補償しないのに実遅延が大きいため、という仮説を
/// プラントで検証する。校正済み励起 (`gen_reception_real`, 実機 σ~220ns 相当) を、実機相当の actuation
/// 遅延 `delay` エッジを入れたプラントに通し、3 構成を比べる:
///   prod  = kp_inv auto(8), smith_edges 1 (本番)
///   damp  = kp_inv 4, smith_edges 1 (減衰強化・遅延補償なし → 発振するはず)
///   damp+ = kp_inv 4, smith_edges `delay` (遅延を Smith に入れる → 安定して wander 低下するはず)
/// 期待: delay≥2 で damp は発散 (sd 増/lock 低)、damp+ は安定し prod より wander 小 = 「遅延を補償すれば
/// 減衰で共振を潰せる」= ユーザ直感の裏付け。実機 PRBS で d を測って確定する前の host 妥当性確認。
///   cargo test -p gnssdo --test control_bench -- --ignored delay_compensation_enables_damping --nocapture
#[test]
#[ignore = "host validation of delay-compensated damping (prints a table)"]
fn delay_compensation_enables_damping() {
    let cfg = |kp_inv: i64, se: u32| PhaseLockLoopConfig {
        i_den: 512,
        d_den: 16,
        kp_inv,
        smith_edges: se,
        ..PhaseLockLoopConfig::DEFAULT
    };
    let (n, warm, seeds) = (4000usize, 400usize, 4u64);
    eprintln!("DELAY-COMP: calibrated excitation (real σ~220ns), production PLL i_den=512/d16");
    eprintln!("  delay  config            sd[ns]  lock%   adjΔsd[ns]");
    for delay in [2usize, 4, 6, 8] {
        for (label, kp, se) in [
            ("prod (kp8,se1)", 0i64, 1u32),
            ("kp4,se1", 4, 1),
            ("kp4,se=d", 4, delay as u32),
            ("kp2,se1", 2, 1),
            ("kp2,se=d", 2, delay as u32),
        ] {
            let (mut sd_s, mut lock_s, mut adj_s) = (0.0, 0.0, 0.0);
            for s in 0..seeds {
                let rec = gen_reception_real(n, 300 + s);
                let mut pll = PhaseLockLoop::with_config(cfg(kp, se));
                let (hw, lk) =
                    run_plant_delayed(&mut pll, n, &rec, PIO, 0.0, 0.0, 7000 + s, delay);
                let h = &hw[warm..];
                sd_s += std_dev(h);
                lock_s += lk[warm..].iter().filter(|&&x| x).count() as f64 / (n - warm) as f64;
                let adj: Vec<f64> = h.windows(2).map(|w| w[1] - w[0]).collect();
                adj_s += std_dev(&adj);
            }
            eprintln!(
                "  {:>4}   {:<16}  {:6.0}  {:5.0}   {:8.0}",
                delay,
                label,
                sd_s / seeds as f64,
                100.0 * lock_s / seeds as f64,
                adj_s / seeds as f64
            );
        }
    }
}

/// **多エッジ Smith が実遅延下で減衰を取り戻す回帰** (本丸 #7 の中核を pin)。実機相当の ~8 エッジ出力
/// パイプライン遅延では、P 減衰を上げる (kp_inv 8→4) と 1-edge Smith では**発散**する (実機で見た
/// 「kp_inv=4 発振」の再現)。`smith_edges` を遅延に合わせると安定化し、wander は production 以下になる
/// = 遅延を補償すれば減衰で共振を潰せる。これがライブラリの不変条件 (汎用: 遅延補償した減衰は安定)。
#[test]
fn multi_edge_smith_restores_damping_under_realistic_delay() {
    let delay = 8usize;
    let run = |kp_inv: i64, se: u32| -> (f64, f64) {
        let (mut sd, mut lock) = (0.0, 0.0);
        let seeds = 3u64;
        for s in 0..seeds {
            let rec = gen_reception_real(4000, 300 + s);
            let mut pll = PhaseLockLoop::with_config(PhaseLockLoopConfig {
                i_den: 512,
                d_den: 16,
                kp_inv,
                smith_edges: se,
                ..PhaseLockLoopConfig::DEFAULT
            });
            let (hw, lk) = run_plant_delayed(&mut pll, 4000, &rec, PIO, 0.0, 0.0, 7000 + s, delay);
            sd += std_dev(&hw[400..]);
            lock += lk[400..].iter().filter(|&&x| x).count() as f64 / 3600.0;
        }
        (sd / seeds as f64, 100.0 * lock / seeds as f64)
    };
    let (prod_sd, prod_lock) = run(0, 1); // production: kp8 / se1
    let (damp1_sd, damp1_lock) = run(4, 1); // stronger damping, 1-edge Smith → unstable
    let (damp8_sd, damp8_lock) = run(4, delay as u32); // + multi-edge Smith → stable
    assert!(prod_lock > 90.0, "production sanity: lock={prod_lock:.0}%");
    assert!(
        damp1_lock < 50.0 || damp1_sd > 5_000.0,
        "kp4/se1 should be unstable at delay {delay}: sd={damp1_sd:.0} lock={damp1_lock:.0}%"
    );
    assert!(
        damp8_lock > 95.0 && damp8_sd < 1_000.0,
        "multi-edge Smith should re-stabilize: sd={damp8_sd:.0} lock={damp8_lock:.0}%"
    );
    assert!(
        damp8_sd <= prod_sd + 20.0,
        "delay-compensated damping should not exceed production wander: damp8={damp8_sd:.0} prod={prod_sd:.0}"
    );
}
