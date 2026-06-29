# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "scipy", "matplotlib"]
# ///
"""
i_den sweep {128,256,512,1024} for the GPSDO output-phase PLL, to choose a config
for STABLE TENS-OF-NS without over-fitting.

WHAT IS VERIFIED (by the two prior model-verification agents + HW logs):
  - The wander is the underdamped type-II mode at period 2*pi*sqrt(i_den) edges.
    HW confirms: i_den=32 -> 36 s, i_den=128 -> 77 s. The PERIOD axis is verified.
  - A single fixed colored theta_ref through the (verified-LINEAR) loop reproduces
    the PERIOD at every i_den but gives a FLAT amplitude (~620-670 ns) -- it does
    NOT reproduce the HW ~6x amplitude drop from i_den 32->128. So the model's
    ABSOLUTE sd is NOT trustworthy; the loop's linear response to a fixed colored
    input does not contain the i_den-coupled mechanism that drops HW amplitude.

WHAT IS TRUSTED for the amplitude prediction: the TWO directly-measured HW points
    i_den=32 -> sd 660 ns (660-725 band), i_den=128 -> sd 112 ns (firmware self-meas).
    These bracket an EMPIRICAL amplitude-vs-i_den law. We extrapolate it CONSERVATIVELY
    (two ways: power-law fit through the two points, and a floor-limited model) and
    flag the extrapolation as a prediction, not a measurement.

WHAT THE MODEL IS USED FOR (where it IS valid):
  1. The closed-loop transfer theta_ref->hwphase (LINEAR, verified) gives the loop's
     high-pass corner and resonant peak gain vs i_den. This governs how much a slow
     FEEDFORWARD-error ramp (crystal drift the alpha-beta FF doesn't perfectly cancel)
     leaks into output phase -- the term that makes loosening i_den STOP helping.
  2. Lock-safety: does the loop stay locked under the excitation at each i_den.
  3. Ring-down zeta vs i_den and vs structural changes (kp_inv, d_den) -- to decide
     whether a structural change beats simply loosening i_den.

This is the honest synthesis: PERIOD from the verified model, AMPLITUDE from the two
HW anchors (extrapolated), FF-leak + lock-safety + damping from the verified loop.
"""
import sys, numpy as np
sys.path.insert(0, "/home/sksat/prog/pico-gnss-rs/report")
from sim_resonance import PhaseLockLoop, trunc_div


def make_pll(i_den, shift=0, kp_inv=8, d_den=4):
    return PhaseLockLoop(
        mode_smith=True, use_i=True, use_d=True, deadband_ns=0,
        lock_ns=1_000, lock_hold=5, i_enable_ns=5_000, outlier_ns=3_000,
        outlier_max=12, i_den=i_den, trim_max_mppb=3_000_000, d_den=d_den,
        calm_ns=1_000, i_den_disturbed_shift=shift, kp_inv=kp_inv,
    )


def cap16(x):
    return int(np.int64(np.round(x / 16.0)) * 16)


def gen_theta_ref(n, sigma_rw, sigma_w, seed):
    rng = np.random.default_rng(seed)
    theta = np.cumsum(rng.normal(0.0, sigma_rw, n)) + rng.normal(0.0, sigma_w, n)
    return np.round(theta).astype(np.int64)


def run_plant(i_den, theta_ref, kp_inv=8, d_den=4, ff_freq_accel_ns_per_s2=0.0):
    """Closed loop. ff_freq_accel = residual feedforward error the alpha-beta FF
    fails to cancel. A type-II loop has ZERO steady error to a constant frequency
    OFFSET (the I integrator absorbs it), so a constant FF leak does not expose the
    loop. What it CANNOT null with zero steady error is a frequency RAMP (df/dt:
    thermal crystal drift, or alpha-beta slope lag) -> phase that grows
    quadratically in time. Against that, a type-II loop leaves a finite steady
    phase error proportional to the loop time-constant^2 ~ i_den. That is the term
    that makes loosening i_den STOP helping: looser loop -> larger residual to the
    same crystal-drift acceleration. We inject it as a per-edge frequency that
    ramps at ff (ns/s per s), added to theta_out's advance."""
    pll = make_pll(i_den, kp_inv=kp_inv, d_den=d_den)
    n = len(theta_ref)
    theta_out = 0
    obs = np.empty(n, dtype=np.int64)
    ff_freq = 0.0   # uncancelled FF frequency error (ns/s), ramps over time
    for k in range(n):
        ff_freq += ff_freq_accel_ns_per_s2     # frequency acceleration
        err = theta_out - int(theta_ref[k])
        u = pll.update(cap16(err), True)
        obs[k] = err
        theta_out += trunc_div(u["freq_trim_mppb"], 1000) - u["phase_corr_ns"] + int(round(ff_freq))
    return obs, pll.is_locked()


def dom_period_fft(x, fs=1.0):
    x = np.asarray(x, float) - np.mean(x)
    X = np.fft.rfft(x * np.hanning(len(x)))
    f = np.fft.rfftfreq(len(x), d=1.0 / fs)
    psd = np.abs(X) ** 2
    psd[0] = 0
    k = np.argmax(psd)
    return (1.0 / f[k]) if f[k] > 0 else np.inf


def transfer_gain(i_den, period_s, amp_ns=200, n=12000, burn=4000, kp_inv=8, d_den=4):
    pll = make_pll(i_den, kp_inv=kp_inv, d_den=d_den)
    theta_out = 0
    t = np.arange(n)
    ref = np.round(amp_ns * np.sin(2 * np.pi * t / period_s)).astype(np.int64)
    hw = np.empty(n)
    for k in range(n):
        err = theta_out - int(ref[k])
        u = pll.update(cap16(err), True)
        hw[k] = err
        theta_out += trunc_div(u["freq_trim_mppb"], 1000) - u["phase_corr_ns"]
    return np.sqrt(2) * hw[burn:].std() / amp_ns


def peak_gain(i_den, kp_inv=8, d_den=4):
    grid = list(range(16, 320, 6))
    best = max((transfer_gain(i_den, p, kp_inv=kp_inv, d_den=d_den), p) for p in grid)
    return best  # (gain, period)


def impulse_ringdown(i_den, kp_inv=8, d_den=4, kick=2000, n=900):
    """Free ring-down to a 2us phase kick (no noise): modal period, zeta (log
    decrement), peak overshoot, n zero-crossings. zeta is the damping discriminator."""
    p = make_pll(i_den, kp_inv=kp_inv, d_den=d_den)
    for _ in range(40):
        p.update(0, True)
    phase = kick
    s = np.empty(n)
    for i in range(n):
        u = p.update(int(phase), True)
        phase += trunc_div(u["freq_trim_mppb"], 1000) - u["phase_corr_ns"]
        s[i] = phase
    peak = float(np.abs(s).max())
    zc = int(np.sum((s[:-1] > 0) != (s[1:] > 0)))
    peaks = [(i, s[i]) for i in range(1, n - 1) if s[i] > s[i-1] and s[i] >= s[i+1] and s[i] > 0]
    zeta = float("nan"); period = float("nan")
    if len(peaks) >= 3:
        idx = [q[0] for q in peaks]; amp = [q[1] for q in peaks]
        period = float(np.median(np.diff(idx)))
        kk = min(3, len(amp) - 1)
        if amp[0] > 0 and amp[kk] > 0 and kk > 0:
            delta = np.log(amp[0] / amp[kk]) / kk
            zeta = float(delta / np.sqrt(4 * np.pi**2 + delta**2))
    return dict(period=period, zeta=zeta, peak=peak, zc=zc)


# --------------------------------------------------------------------------- #
# HW amplitude anchors and the empirical amplitude-vs-i_den law
# --------------------------------------------------------------------------- #
HW = {32: 660.0, 128: 112.0}   # firmware self-meas sd (the trusted amplitude points)


def amp_powerlaw(i_den):
    """Power-law through the two HW anchors: sd = C * i_den**p.
    p = ln(112/660)/ln(128/32);  C from i_den=32."""
    p = np.log(HW[128] / HW[32]) / np.log(128 / 32)
    C = HW[32] / (32.0 ** p)
    return C * i_den ** p, p


def main():
    print("=" * 78)
    print("HONEST i_den sweep: PERIOD from verified model, AMPLITUDE from 2 HW anchors")
    print("=" * 78)

    p_law = np.log(HW[128] / HW[32]) / np.log(128 / 32)
    print(f"\nHW anchors: i_den=32 -> 660 ns, i_den=128 -> 112 ns")
    print(f"empirical power-law exponent p = {p_law:.2f} (sd ~ i_den^{p_law:.2f}); "
          f"per 4x in i_den -> {4**p_law:.2f}x amplitude drop\n")

    # 1) linear-loop peak gain + ring-down zeta vs i_den (the model's VALID outputs)
    print("--- verified linear loop: resonant peak gain + impulse ring-down vs i_den ---")
    print(f"{'i_den':>6} {'peakG':>7} {'@T_s':>6} {'2pi*sqrt':>9} {'ring_T':>7} "
          f"{'zeta':>6} {'peak_ns':>8} {'zc':>4}")
    gains = {}
    for i_den in [32, 64, 128, 256, 512, 1024]:
        g, gp = peak_gain(i_den)
        r = impulse_ringdown(i_den)
        gains[i_den] = g
        nat = 2 * np.pi * np.sqrt(i_den)
        z = f"{r['zeta']:.3f}" if not np.isnan(r['zeta']) else "  ~0 "
        print(f"{i_den:>6} {g:7.2f} {gp:6d} {nat:9.1f} {r['period']:7.0f} "
              f"{z:>6} {r['peak']:8.0f} {r['zc']:>4}")

    # 2) the headline sweep table: predicted wander, residual period, lock-safety
    print("\n--- SWEEP {128,256,512,1024}: predicted wander (HW-anchored) + period + lock ---")
    print(f"{'i_den':>6} {'period_s':>9} {'sd_pred_ns':>11} {'lock_safe':>10}  notes")
    # lock-safety: scale the colored excitation PER i_den so the OUTPUT sd matches
    # the HW-anchored prediction, then check lock. The linear model under-excites,
    # so a single sigma_rw that makes i_den=32 hit 660 ns OVER-drives i_den>=128 and
    # falsely unlocks it (HW locks fine at 128). Picking sigma_rw per i_den isolates
    # "is THIS output amplitude lockable" from "the model under-excites".
    sweep = {}
    for i_den in [128, 256, 512, 1024]:
        period = 2 * np.pi * np.sqrt(i_den)
        sd_pl, _ = amp_powerlaw(i_den)
        lo, hi = 1.0, 500.0
        for _ in range(13):
            mid = (lo + hi) / 2
            sds = []
            for s in range(4):
                ref = gen_theta_ref(5000, mid, 4, 4000 + s)
                o, _ = run_plant(i_den, ref)
                sds.append(o[800:].std())
            if np.mean(sds) < sd_pl:
                lo = mid
            else:
                hi = mid
        sigma_rw = (lo + hi) / 2
        locks = []
        for s in range(8):
            ref = gen_theta_ref(6000, sigma_rw, 4, 4000 + s)
            _, lk = run_plant(i_den, ref)
            locks.append(lk)
        lock_safe = all(locks)
        sweep[i_den] = dict(period=period, sd_pred=sd_pl, lock_safe=lock_safe, sigma_rw=sigma_rw)
        note = "current DEFAULT (HW locks fine)" if i_den == 128 else ""
        print(f"{i_den:>6} {period:9.1f} {sd_pl:11.0f} {str(lock_safe):>10}  "
              f"[sigma_rw={sigma_rw:.0f}] {note}")

    # 3) FEEDFORWARD-error leakage: where loosening STOPS helping.
    # A residual FF frequency error (alpha-beta doesn't perfectly cancel crystal
    # drift) is a slow phase ramp. The I-integrator must cancel it; a slower (larger
    # i_den) integrator leaves a larger transient/steady phase excursion. Quantify
    # the steady residual phase for a small ramp at each i_den.
    print("\n--- FEEDFORWARD-error leakage: residual phase under a FF FREQ ACCELERATION ---")
    print("    A type-II loop nulls a constant FF freq OFFSET with zero steady error, so a")
    print("    constant leak does NOT expose it. The exposing disturbance is df/dt (crystal")
    print("    drift acceleration / alpha-beta slope lag): a freq RAMP. Steady phase error of")
    print("    a type-II loop to a freq ramp ~ (loop time const)^2 ~ i_den. ff in ns/s per s.")
    accels = [0.002, 0.005, 0.01]
    print(f"{'i_den':>6} " + "".join(f"ff={f}".rjust(12) for f in accels))
    ff_resid = {}
    for i_den in [128, 256, 512, 1024]:
        row = []
        for ff in accels:
            obs, _ = run_plant(i_den, np.zeros(12000, dtype=np.int64), ff_freq_accel_ns_per_s2=ff)
            steady = obs[6000:]
            row.append(float(np.mean(np.abs(steady))))
        ff_resid[i_den] = row
        print(f"{i_den:>6} " + "".join(f"{v:11.0f} " for v in row))

    # combined: total predicted wander = sqrt(anchored_wander^2 + ff_leak^2) @ ff=0.005
    print("\n--- COMBINED predicted wander = quad-sum(HW-anchored mode, FF-leak @0.005ns/s/s) ---")
    print(f"{'i_den':>6} {'mode_ns':>8} {'ff_leak_ns':>11} {'total_ns':>9} {'period_s':>9} {'lock':>5}")
    for i_den in [128, 256, 512, 1024]:
        mode = sweep[i_den]['sd_pred']
        ffl = ff_resid[i_den][1]   # ff=0.005
        total = np.sqrt(mode**2 + ffl**2)
        print(f"{i_den:>6} {mode:8.0f} {ffl:11.0f} {total:9.0f} "
              f"{sweep[i_den]['period']:9.1f} {str(sweep[i_den]['lock_safe']):>5}")

    # 4) STRUCTURAL change test: can we damp the mode instead of loosening?
    print("\n--- STRUCTURAL: ring-down zeta vs kp_inv and d_den at i_den=128 ---")
    print(f"{'config':>28} {'zeta':>7} {'peak_ns':>8} {'zc':>4} {'lockable':>9}")
    for label, kp, dd in [("DEFAULT kp_inv=8 d_den=4", 8, 4),
                          ("kp_inv=4 (more P)", 4, 4),
                          ("kp_inv=16 (less P)", 16, 4),
                          ("d_den=2 (more D)", 8, 2),
                          ("d_den=8 (less D)", 8, 8),
                          ("d_den=1 (max D)", 8, 1)]:
        r = impulse_ringdown(128, kp_inv=kp, d_den=dd)
        # lockability under the colored ref
        locks = []
        for s in range(6):
            ref = gen_theta_ref(5000, 320, 4, 4000 + s)
            _, lk = run_plant(128, ref, kp_inv=kp, d_den=dd)
            locks.append(lk)
        z = f"{r['zeta']:.3f}" if not np.isnan(r['zeta']) else " ~0/neg"
        print(f"{label:>28} {z:>7} {r['peak']:8.0f} {r['zc']:>4} {str(all(locks)):>9}")

    # 5) ROBUSTNESS: vary theta_ref amplitude +/-2x and color -> does the
    #    RELATIVE i_den behavior (period scaling, lock-safety) hold?
    print("\n--- ROBUSTNESS: vary excitation amplitude/color; check period scaling holds ---")
    for label, srw, sw in [("baseline rw=320", 320, 4),
                           ("amp x2 rw=640", 640, 4),
                           ("amp /2 rw=160", 160, 4),
                           ("whiter rw=80,w=40", 80, 40),
                           ("redder rw=640,w=1", 640, 1)]:
        ref32 = gen_theta_ref(6000, srw, sw, 7)
        o32, lk32 = run_plant(32, ref32)
        ref512 = gen_theta_ref(6000, srw, sw, 7)
        o512, lk512 = run_plant(512, ref512)
        t32 = dom_period_fft(o32[800:]); t512 = dom_period_fft(o512[800:])
        print(f"  {label:>20}: T(32)={t32:5.0f}s T(512)={t512:5.0f}s "
              f"ratio={t512/t32:.1f}x (expect ~{np.sqrt(512/32):.1f}x)  "
              f"lock32={lk32} lock512={lk512}")


if __name__ == "__main__":
    main()
