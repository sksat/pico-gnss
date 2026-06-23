# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy"]
# ///
"""
Independent head-to-head of the three candidate phase-loop designs, all run
through the SAME closed-loop plant and the SAME step / noise / drift harness, so
the comparison is apples-to-apples and not three differently-instrumented
prototype reports.

Ported EXACTLY from the Rust I read (gnssdo/src/pll.rs in each worktree):
  - PRODUCTION  PidSmith  i_den=128 d_den=4 kp_inv=8 (DEFAULT)
  - joint-gain  PidSmith  i_den=512 d_den=16 kp_inv=8  (worktree -718-1)
  - smith-2edge PidSmith  smith_edges=2 kp_inv=4 d_den=16 i_den=256 (worktree -718-2)
  - kalman      AlphaBetaTracker alpha=124/1024 beta=4/1024 (worktree -718-3)

Integer-only, Rust truncating division everywhere.
"""
import numpy as np

CORR_CLAMP_NS = 100_000_000


def td(a, b):  # Rust i64 trunc-toward-zero division
    a = int(a); b = int(b)
    q = abs(a) // abs(b)
    return -q if (a < 0) != (b < 0) else q


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


# --------------------------------------------------------------------------
# PID loop with a configurable multi-edge Smith predictor (covers production,
# joint-gain, and smith-2edge by parameters).
# --------------------------------------------------------------------------
class PidSmith:
    def __init__(self, i_den=128, d_den=4, kp_inv=8, smith_edges=1,
                 lock_ns=1000, lock_hold=5, i_enable_ns=5000,
                 outlier_ns=3000, outlier_max=12, trim_max=3_000_000):
        self.i_den = i_den; self.d_den = d_den; self.kp_inv = kp_inv
        self.smith_edges = smith_edges
        self.lock_ns = lock_ns; self.lock_hold = lock_hold
        self.i_enable_ns = i_enable_ns
        self.outlier_ns = outlier_ns; self.outlier_max = outlier_max
        self.trim_max = trim_max
        self.lock_cnt = 0; self.reject_cnt = 0; self.trim = 0
        self.last_pred = 0
        self.pd_inflight = [0] * smith_edges  # ring of last applied p+d corrections

    def locked(self):
        return self.lock_cnt >= self.lock_hold

    def update(self, phase, valid=True):
        ctrl = int(phase)
        pred = ctrl - sum(self.pd_inflight)  # Smith subtracts in-flight P+D (sum over edges)
        locked = self.locked()
        p_corr = d_corr = 0
        applied = False
        if valid:
            if locked and abs(ctrl) > self.outlier_ns and self.reject_cnt < self.outlier_max:
                self.reject_cnt += 1
            else:
                self.reject_cnt = 0
                applied = True
                if abs(ctrl) < self.i_enable_ns:
                    self.trim = clamp(self.trim - td(pred * 1000, self.i_den),
                                      -self.trim_max, self.trim_max)
                p_corr = clamp(td(pred, self.kp_inv), -CORR_CLAMP_NS, CORR_CLAMP_NS)
                if locked:
                    d_corr = clamp(td(pred - self.last_pred, self.d_den),
                                   -CORR_CLAMP_NS, CORR_CLAMP_NS)
                self.lock_cnt = min(self.lock_cnt + 1, self.lock_hold) if abs(ctrl) < self.lock_ns else 0
        if applied:
            self.last_pred = pred
            self.pd_inflight = (self.pd_inflight + [p_corr + d_corr])[1:]
        return {"trim": self.trim, "corr": p_corr + d_corr, "applied": applied,
                "locked": locked}


# --------------------------------------------------------------------------
# AlphaBetaTracker (exact port of worktree -718-3)
# --------------------------------------------------------------------------
class AlphaBeta:
    def __init__(self, alpha_num=124, alpha_den=1024, beta_num=4, beta_den=1024,
                 lock_ns=1000, lock_hold=5, outlier_ns=3000, outlier_max=12,
                 trim_max=3_000_000):
        self.an, self.ad, self.bn, self.bd = alpha_num, alpha_den, beta_num, beta_den
        self.lock_ns = lock_ns; self.lock_hold = lock_hold
        self.outlier_ns = outlier_ns; self.outlier_max = outlier_max
        self.trim_max = trim_max
        self.phase = 0; self.freq = 0
        self.prev_corr = 0; self.prev_trim = 0
        self.lock_cnt = 0; self.reject_cnt = 0

    def locked(self):
        return self.lock_cnt >= self.lock_hold

    def update(self, phase_err, valid=True):
        locked = self.locked()
        phase_pred = self.phase + td(self.freq, 1000) + td(self.prev_trim, 1000) - self.prev_corr
        if (not valid) or (locked and abs(phase_err) > self.outlier_ns
                           and self.reject_cnt < self.outlier_max):
            if valid:
                self.reject_cnt += 1
            ft = -self.freq
            self.phase = phase_pred
            self.prev_corr = 0
            self.prev_trim = ft
            return {"trim": ft, "corr": 0, "applied": False, "locked": locked}
        self.reject_cnt = 0
        r = phase_err - phase_pred
        self.phase = phase_pred + td(r * self.an, self.ad)
        self.freq = clamp(self.freq + td(r * self.bn * 1000, self.bd), -self.trim_max, self.trim_max)
        corr = clamp(self.phase, -CORR_CLAMP_NS, CORR_CLAMP_NS)
        ft = -self.freq
        self.prev_corr = corr
        self.prev_trim = ft
        self.lock_cnt = min(self.lock_cnt + 1, self.lock_hold) if abs(phase_err) < self.lock_ns else 0
        return {"trim": ft, "corr": corr, "applied": True, "locked": locked}


# --------------------------------------------------------------------------
# Shared closed-loop plant (1-edge actuation delay is implicit: corr/trim issued
# this edge are applied to the phase, and the Smith/prev_corr bookkeeping models
# the in-flight delay).  plant_delay lets us stress an UNDER-modelled delay.
# --------------------------------------------------------------------------
def step_ringdown(make, step_ns=300, n=600, warm=60, plant_delay=1):
    pll = make()
    for _ in range(warm):
        pll.update(0, True)
    phase = step_ns
    cmd_q = [0] * plant_delay  # extra delay queue beyond the loop's own 1-edge model
    series = []
    for _ in range(n):
        u = pll.update(int(phase), True)
        cmd = td(u["trim"], 1000) - u["corr"]
        cmd_q.append(cmd)
        phase += cmd_q.pop(0)
        series.append(phase)
    s = np.array(series, dtype=float)
    # metrics on the ring-down
    peak = float(np.abs(s).max())
    zc = int(np.sum(np.sign(s[:-1]) != np.sign(s[1:])))
    # transient zero-crossings: only while |phase| still > 2x the limit-cycle floor
    floor = float(np.abs(s[-100:]).max()) if len(s) > 100 else 0.0
    thr = max(2 * floor, 20)
    mask = np.abs(s) > thr
    zc_trans = int(np.sum((np.sign(s[:-1]) != np.sign(s[1:])) & mask[:-1]))
    # overshoot: most-negative excursion as a fraction of the step (step is +)
    overshoot = max(0.0, -float(s.min())) / step_ns
    # settle edge: last index where |phase| > thr
    idx = np.where(np.abs(s) > thr)[0]
    settle = int(idx[-1]) + 1 if len(idx) else 0
    # zeta via log-decrement of successive same-sign peaks (nan if no clear ring)
    peaks = [s[i] for i in range(1, len(s) - 1) if s[i] > s[i - 1] and s[i] >= s[i + 1] and s[i] > thr]
    if len(peaks) >= 2 and peaks[0] > 0 and peaks[-1] > 0:
        k = len(peaks) - 1
        delta = np.log(peaks[0] / peaks[-1]) / k
        zeta = float(delta / np.sqrt(4 * np.pi ** 2 + delta ** 2))
    else:
        zeta = float("nan")  # no oscillatory peaks => over/critically damped
    return dict(peak=peak, zc=zc, zc_trans=zc_trans, overshoot=overshoot,
                settle=settle, floor=floor, zeta=zeta)


def noise_drift_run(make, n=8000, white_half=16, slow_amp=0, slow_period=300, seed=0):
    """White per-edge phase noise (half=16 -> sd~9.2; we also run 11/13/19) plus an
    optional slow correlated sinusoidal drift on the controller input. Returns
    output-phase sd, max excursion, locked-fraction over the steady portion."""
    rng = np.random.default_rng(seed)
    white = rng.integers(-white_half, white_half + 1, size=n)
    t = np.arange(n)
    slow = np.round(slow_amp * np.sin(2 * np.pi * t / slow_period)).astype(np.int64) if slow_amp else np.zeros(n, np.int64)
    noise = white + slow
    pll = make()
    phase = 0
    out = np.empty(n, np.int64)
    lk = np.empty(n, bool)
    for k in range(n):
        u = pll.update(int(phase + noise[k]), True)
        phase += td(u["trim"], 1000) - u["corr"]
        out[k] = phase
        lk[k] = pll.locked()
    burn = 1000
    s = out[burn:].astype(float)
    return dict(sd=float(s.std()), maxabs=float(np.abs(s).max()),
                locked_frac=float(lk[burn:].mean()))


def resonant_excitation(make, n=6000, kick_period=400, kick_amp=300, seed=1):
    """Excite at the loop's resonance band: a slow square-wave phase disturbance
    (period ~ a natural period) plus white noise. This is the test that actually
    separates a damped loop from an underdamped one (pure white under-excites)."""
    rng = np.random.default_rng(seed)
    white = rng.integers(-13, 14, size=n)
    t = np.arange(n)
    dist = kick_amp * np.sign(np.sin(2 * np.pi * t / kick_period))
    pll = make()
    phase = 0
    out = np.empty(n, np.int64)
    lk = np.empty(n, bool)
    for k in range(n):
        u = pll.update(int(phase + white[k] + int(dist[k])), True)
        phase += td(u["trim"], 1000) - u["corr"]
        out[k] = phase
        lk[k] = pll.locked()
    s = out[1000:].astype(float)
    return dict(sd=float(s.std()), maxabs=float(np.abs(s).max()),
                locked_frac=float(lk[1000:].mean()))


DESIGNS = {
    "production":  lambda: PidSmith(i_den=128, d_den=4, kp_inv=8, smith_edges=1),
    "joint-gain":  lambda: PidSmith(i_den=512, d_den=16, kp_inv=8, smith_edges=1),
    "smith-2edge": lambda: PidSmith(i_den=256, d_den=16, kp_inv=4, smith_edges=2),
    "kalman-2st":  lambda: AlphaBeta(),
}


def main():
    print("=" * 78)
    print("STEP RING-DOWN (300 ns step, plant_delay=1 = the loop's own delay model)")
    print("=" * 78)
    print(f"{'design':12s} {'peak':>6s} {'zc':>4s} {'zc_tr':>5s} {'oversh':>7s} "
          f"{'settle':>6s} {'floor':>6s} {'zeta':>6s}")
    for name, make in DESIGNS.items():
        r = step_ringdown(make, 300, plant_delay=1)
        zs = f"{r['zeta']:.2f}" if not np.isnan(r['zeta']) else "  od"
        print(f"{name:12s} {r['peak']:6.0f} {r['zc']:4d} {r['zc_trans']:5d} "
              f"{r['overshoot']*100:6.0f}% {r['settle']:6d} {r['floor']:6.0f} {zs:>6s}")

    print()
    print("STEP RING-DOWN (2000 ns step) — bigger excitation")
    print(f"{'design':12s} {'peak':>6s} {'zc':>4s} {'zc_tr':>5s} {'oversh':>7s} {'settle':>6s}")
    for name, make in DESIGNS.items():
        r = step_ringdown(make, 2000, plant_delay=1)
        print(f"{name:12s} {r['peak']:6.0f} {r['zc']:4d} {r['zc_trans']:5d} "
              f"{r['overshoot']*100:6.0f}% {r['settle']:6d}")

    print()
    print("STEP RING-DOWN ROBUSTNESS — plant_delay=2 and 3 (UNDER-MODELLED actuation delay)")
    for pd in (2, 3):
        print(f"  -- plant_delay={pd} (300 ns step) --")
        for name, make in DESIGNS.items():
            r = step_ringdown(make, 300, plant_delay=pd)
            zs = f"{r['zeta']:.2f}" if not np.isnan(r['zeta']) else "od"
            print(f"    {name:12s} peak={r['peak']:6.0f} zc={r['zc']:3d} "
                  f"zc_tr={r['zc_trans']:3d} oversh={r['overshoot']*100:4.0f}% "
                  f"settle={r['settle']:4d} zeta={zs}")

    print()
    print("=" * 78)
    print("WHITE NOISE + SLOW DRIFT (8000 edges) — does it stay LOCKED + bounded?")
    print("=" * 78)
    for half, amp in [(11, 0), (16, 0), (19, 0), (16, 30), (16, 60)]:
        tag = f"white+/-{half}" + (f" +slow{amp}ns/300s" if amp else "")
        print(f"  -- {tag} --")
        for name, make in DESIGNS.items():
            agg = []
            for seed in range(4):
                agg.append(noise_drift_run(make, white_half=half, slow_amp=amp, seed=seed))
            sd = np.mean([a["sd"] for a in agg])
            mx = np.max([a["maxabs"] for a in agg])
            lk = np.min([a["locked_frac"] for a in agg])
            print(f"    {name:12s} sd={sd:5.1f}ns  max={mx:6.0f}ns  locked={lk*100:5.1f}%")

    print()
    print("=" * 78)
    print("RESONANT EXCITATION (square-wave disturbance near a natural period + noise)")
    print("  -- this is what actually separates damped from underdamped --")
    print("=" * 78)
    for kp in (300, 400, 600):
        print(f"  -- disturbance period {kp} edges, +/-300 ns square --")
        for name, make in DESIGNS.items():
            agg = [resonant_excitation(make, kick_period=kp, seed=s) for s in range(3)]
            sd = np.mean([a["sd"] for a in agg])
            mx = np.max([a["maxabs"] for a in agg])
            lk = np.min([a["locked_frac"] for a in agg])
            print(f"    {name:12s} sd={sd:6.1f}ns  max={mx:6.0f}ns  locked={lk*100:5.1f}%")


if __name__ == "__main__":
    main()


def stress_worst_case():
    """Adversarial lock-retention proxy: the host model has NO lock window, so a
    design that throws a big transient excursion would have dropped lock on HW.
    Worst case = a large thermal step (1us) + actuation delay under-modelled (=3)
    + continuous 13ns white noise. Report the WORST single-edge excursion after
    the step settles within the first 30 edges (the transient overshoot that a
    real lock window would see) and whether it stays bounded."""
    import numpy as np
    print()
    print("=" * 78)
    print("ADVERSARIAL: 1us step + plant_delay=3 + 13ns white  (lock-window proxy)")
    print("  worst transient excursion in first 40 edges + final bounded sd")
    print("=" * 78)
    for name, make in DESIGNS.items():
        worst = 0.0
        finals = []
        for seed in range(6):
            rng = np.random.default_rng(seed)
            pll = make()
            for _ in range(60):
                pll.update(int(rng.integers(-13, 14)), True)
            phase = 1000
            cmd_q = [0, 0, 0]
            ser = []
            for k in range(800):
                u = pll.update(int(phase + rng.integers(-13, 14)), True)
                cmd_q.append(td(u["trim"], 1000) - u["corr"])
                phase += cmd_q.pop(0)
                ser.append(phase)
            ser = np.array(ser, float)
            worst = max(worst, float(np.abs(ser[:40]).max()))
            finals.append(float(ser[-300:].std()))
        ov = (worst - 1000) / 1000 * 100
        print(f"    {name:12s} worst_excursion={worst:6.0f}ns (overshoot {ov:+4.0f}%)  "
              f"final_sd={np.mean(finals):4.1f}ns")


stress_worst_case()
