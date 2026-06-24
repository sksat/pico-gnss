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

fn gen_reception(n: usize, seed: u64) -> Vec<f64> {
    let mut r = Rng::new(seed);
    let mut walk = 0.0;
    (0..n)
        .map(|_| {
            walk += SIGMA_RW * r.gauss();
            walk + SIGMA_W * r.gauss()
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
