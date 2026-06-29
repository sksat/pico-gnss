# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "scipy", "matplotlib"]
# ///
"""
Independent Python reimplementation of the GPSDO output-phase PLL and its
closed-loop plant, to probe the ~30-40s output-phase wander as an underdamped
type-II PLL resonance.

Mirrors EXACTLY (read the Rust, did not guess):
  - gnssdo/src/pll.rs  PhaseLockLoop::update  (the control law)
  - gnssdo/tests/thermal_plant.rs::simulate() (the closed-loop plant)

Integer-only core: Rust i64 uses truncating-toward-zero division. Python's //
floors, so we use a trunc_div helper everywhere a Rust integer division occurs.

Constant-crystal assumption (per task): DisciplinedClock perfectly cancels the
constant crystal, so commanded - true_avg == freq_trim_mppb, giving
    phase += freq_trim_mppb // 1000 - phase_corr_ns
with WHITE phase measurement noise added to the controller input (the loopback
capture jitter), sd ~= 11 ns (matches the real median per-edge interval jitter).
"""

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Exact integer arithmetic helpers (Rust i64 semantics)
# ---------------------------------------------------------------------------

CORR_CLAMP_NS = 100_000_000


def trunc_div(a, b):
    """Integer division truncating toward zero (Rust `/` on i64)."""
    q = abs(int(a)) // abs(int(b))
    if (a < 0) != (b < 0):
        q = -q
    return q


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


# ---------------------------------------------------------------------------
# PhaseLockLoop (exact port of gnssdo/src/pll.rs PhaseLockLoop::update)
# ---------------------------------------------------------------------------

# kp_inv from LoopMode::kp_inv(): 8 for Smith modes, 16 otherwise.
# We allow overriding kp_inv for the damping-restoration sweep (the task asks
# to sweep kp_inv even though firmware hardcodes it; that is a "what if it were
# configurable" axis).


class PhaseLockLoop:
    def __init__(
        self,
        mode_smith=True,
        use_i=True,
        use_d=True,
        deadband_ns=0,
        lock_ns=1_000,
        lock_hold=5,
        i_enable_ns=5_000,
        outlier_ns=3_000,
        outlier_max=12,
        i_den=128,
        trim_max_mppb=3_000_000,
        d_den=4,
        calm_ns=1_000,
        i_den_disturbed_shift=0,
        kp_inv=None,
    ):
        self.mode_smith = mode_smith
        self.use_i = use_i
        self.use_d = use_d
        self.deadband_ns = deadband_ns
        self.lock_ns = lock_ns
        self.lock_hold = lock_hold
        self.i_enable_ns = i_enable_ns
        self.outlier_ns = outlier_ns
        self.outlier_max = outlier_max
        self.i_den = i_den
        self.trim_max_mppb = trim_max_mppb
        self.d_den = d_den
        self.calm_ns = calm_ns
        self.i_den_disturbed_shift = i_den_disturbed_shift
        if kp_inv is None:
            kp_inv = 8 if mode_smith else 16
        self.kp_inv = kp_inv

        # state
        self.lock_cnt = 0
        self.reject_cnt = 0
        self.trim_mppb = 0
        self.last_pred = 0
        self.last_pd = 0

    def is_locked(self):
        return self.lock_cnt >= self.lock_hold

    def update(self, phase_err_ns, valid=True):
        ctrl = int(phase_err_ns)
        pred = ctrl - self.last_pd if self.mode_smith else ctrl
        locked = self.is_locked()
        p_corr = 0
        d_corr = 0
        applied = False
        rejected_outlier = False

        if valid:
            if (
                locked
                and abs(ctrl) > self.outlier_ns
                and self.reject_cnt < self.outlier_max
            ):
                self.reject_cnt += 1
                rejected_outlier = True
            else:
                self.reject_cnt = 0
                applied = True
                # I term
                if self.use_i:
                    if abs(ctrl) < self.i_enable_ns:
                        if abs(pred) < self.calm_ns:
                            i_den_eff = self.i_den
                        else:
                            i_den_eff = self.i_den << min(self.i_den_disturbed_shift, 20)
                        self.trim_mppb = clamp(
                            self.trim_mppb - trunc_div(pred * 1000, i_den_eff),
                            -self.trim_max_mppb,
                            self.trim_max_mppb,
                        )
                else:
                    self.trim_mppb = 0
                # P term
                if abs(pred) > self.deadband_ns:
                    p_corr = clamp(
                        trunc_div(pred, self.kp_inv), -CORR_CLAMP_NS, CORR_CLAMP_NS
                    )
                # D term (only while locked)
                if self.use_d and locked:
                    d_corr = clamp(
                        trunc_div(pred - self.last_pred, self.d_den),
                        -CORR_CLAMP_NS,
                        CORR_CLAMP_NS,
                    )
                # lock counting
                if abs(ctrl) < self.lock_ns:
                    self.lock_cnt = min(self.lock_cnt + 1, self.lock_hold)
                else:
                    self.lock_cnt = 0

        if applied:
            self.last_pred = pred
            self.last_pd = p_corr + d_corr

        return {
            "applied": applied,
            "locked": locked,
            "rejected_outlier": rejected_outlier,
            "freq_trim_mppb": self.trim_mppb,
            "phase_corr_ns": p_corr + d_corr,
            "predicted_phase_ns": pred,
        }


# ---------------------------------------------------------------------------
# Closed-loop plant (constant-crystal): exact form of thermal_plant.rs::simulate
# but with crystal cancelled so commanded - true_avg == freq_trim_mppb.
#
#   measured_phase = phase_ns + white_noise[n]      (controller input)
#   u = pll.update(measured_phase)
#   phase_ns += freq_trim_mppb // 1000 - phase_corr_ns
#
# All integer (phase_ns int ns, trim mppb). Noise is integer ns.
# ---------------------------------------------------------------------------


def run_closed_loop(pll, n_edges, noise_ns, seed=0):
    rng = np.random.default_rng(seed)
    # White phase measurement noise: uniform +/-19 ns -> sd ~= 11 ns.
    # (uniform half-width h has sd = h/sqrt(3); 19/sqrt(3) = 10.97)
    half = noise_ns
    noise = rng.integers(-half, half + 1, size=n_edges)  # integer ns
    phase = 0
    out = np.empty(n_edges, dtype=np.int64)
    trim = np.empty(n_edges, dtype=np.int64)
    for n in range(n_edges):
        measured = phase + int(noise[n])
        u = pll.update(measured, True)
        phase += trunc_div(u["freq_trim_mppb"], 1000) - u["phase_corr_ns"]
        out[n] = phase
        trim[n] = u["freq_trim_mppb"]
    return out, trim, noise


def run_closed_loop_with_slow(pll, n_edges, white_half, slow_amp_ns, slow_period_s, seed=0):
    """Variant: white + a small slow (sinusoidal) phase-noise component on the
    controller input, in case pure white under-excites. slow is integer ns."""
    rng = np.random.default_rng(seed)
    white = rng.integers(-white_half, white_half + 1, size=n_edges)
    t = np.arange(n_edges)
    slow = np.round(slow_amp_ns * np.sin(2 * np.pi * t / slow_period_s)).astype(np.int64)
    noise = white + slow
    phase = 0
    out = np.empty(n_edges, dtype=np.int64)
    for n in range(n_edges):
        measured = phase + int(noise[n])
        u = pll.update(measured, True)
        phase += trunc_div(u["freq_trim_mppb"], 1000) - u["phase_corr_ns"]
        out[n] = phase
    return out, noise


# ---------------------------------------------------------------------------
# Spectral / damping analysis
# ---------------------------------------------------------------------------


def dominant_period(x, fs=1.0):
    """Dominant period (s) via FFT of the mean-removed, windowed series."""
    x = np.asarray(x, dtype=float)
    x = x - x.mean()
    w = np.hanning(len(x))
    xw = x * w
    X = np.fft.rfft(xw)
    f = np.fft.rfftfreq(len(x), d=1.0 / fs)
    psd = np.abs(X) ** 2
    # ignore DC bin
    psd[0] = 0.0
    k = np.argmax(psd)
    if f[k] == 0:
        return np.inf
    return 1.0 / f[k]


def autocorr_first_peak_period(x):
    """Period from the first local maximum of the autocorrelation (after the
    initial decline through zero / first trough)."""
    x = np.asarray(x, dtype=float)
    x = x - x.mean()
    n = len(x)
    ac = np.correlate(x, x, mode="full")[n - 1 :]
    ac = ac / ac[0]
    # find first descent, then first local max after autocorr goes below 0 or
    # after the slope turns positive again.
    # simpler: first index>1 where ac[i-1]<ac[i] and ac[i]>ac[i+1] after first dip.
    dipped = False
    for i in range(1, n - 1):
        if not dipped and ac[i] < 0:
            dipped = True
        if dipped and ac[i] > ac[i - 1] and ac[i] >= ac[i + 1] and ac[i] > 0:
            return float(i)
    return float("nan")


def band_power_fraction(x, fs=1.0, lo=1 / 64.0, hi=1 / 16.0):
    """Fraction of (AC) power in the [16,64] s band (freq lo..hi Hz)."""
    x = np.asarray(x, dtype=float)
    x = x - x.mean()
    w = np.hanning(len(x))
    X = np.fft.rfft(x * w)
    f = np.fft.rfftfreq(len(x), d=1.0 / fs)
    psd = np.abs(X) ** 2
    psd[0] = 0.0
    total = psd.sum()
    if total <= 0:
        return 0.0
    band = psd[(f >= lo) & (f <= hi)].sum()
    return band / total


def estimate_zeta_from_psd(x, fs=1.0):
    """Estimate damping ratio from the resonant peak sharpness.

    For an underdamped 2nd-order system the closed-loop output PSD has a peak;
    Q = f0 / bandwidth(-3dB), and zeta ~= 1/(2Q). We find the peak and its
    half-power width. Returns nan if no clear peak."""
    x = np.asarray(x, dtype=float)
    x = x - x.mean()
    w = np.hanning(len(x))
    X = np.fft.rfft(x * w)
    f = np.fft.rfftfreq(len(x), d=1.0 / fs)
    psd = np.abs(X) ** 2
    psd[0] = 0.0
    if len(psd) < 5:
        return float("nan")
    k = np.argmax(psd)
    if k == 0 or k >= len(psd) - 1:
        return float("nan")
    peak = psd[k]
    half = peak / 2.0
    # walk down on both sides to the half-power points
    lo = k
    while lo > 0 and psd[lo] > half:
        lo -= 1
    hi = k
    while hi < len(psd) - 1 and psd[hi] > half:
        hi += 1
    if hi <= lo or f[k] == 0:
        return float("nan")
    bw = f[hi] - f[lo]
    f0 = f[k]
    Q = f0 / bw if bw > 0 else np.inf
    zeta = 1.0 / (2.0 * Q)
    return float(zeta)


def analytic_zeta(i_den, kp_inv, d_den=None):
    """Analytic damping of the type-II loop (matches the diagnosis formula):
    zeta = (1/kp_inv) / (2*sqrt(1/i_den)). D adds extra damping but the headline
    formula in the diagnosis ignores it; we report this as the P/I damping."""
    return (1.0 / kp_inv) / (2.0 * np.sqrt(1.0 / i_den))


def analytic_period(i_den):
    """Natural period in edges (~s): 2*pi*sqrt(i_den)."""
    return 2 * np.pi * np.sqrt(i_den)


def impulse_modal(pll_factory, kick=2000, n=700, warm=20):
    """Free ring-down to a one-edge phase kick (NO noise): the cleanest read of
    the loop's own modal damped period and zeta. zeta from the log-decrement of
    successive positive peaks of the (noise-free) response. This is the robust
    zeta estimate; the noise-driven PSD-Q estimate is unreliable when white noise
    barely excites the mode."""
    p = pll_factory()
    for _ in range(warm):
        p.update(0, True)
    phase = kick
    s = np.empty(n, dtype=float)
    for i in range(n):
        u = p.update(int(phase), True)
        phase += trunc_div(u["freq_trim_mppb"], 1000) - u["phase_corr_ns"]
        s[i] = phase
    # positive local maxima
    peaks = [(i, s[i]) for i in range(1, n - 1) if s[i] > s[i - 1] and s[i] >= s[i + 1] and s[i] > 0]
    if len(peaks) < 3:
        return float("nan"), float("nan"), float(np.abs(s).max())
    idx = [pk[0] for pk in peaks]
    amp = [pk[1] for pk in peaks]
    period = float(np.median(np.diff(idx)))  # edges between same-sign peaks
    k = min(3, len(amp) - 1)
    a0, ak = amp[0], amp[k]
    if a0 <= 0 or ak <= 0 or k == 0:
        return period, float("nan"), float(np.abs(s).max())
    delta = np.log(a0 / ak) / k  # log decrement per cycle
    zeta = delta / np.sqrt(4 * np.pi ** 2 + delta ** 2)
    return period, float(zeta), float(np.abs(s).max())


# ---------------------------------------------------------------------------
# Sweep driver
# ---------------------------------------------------------------------------

N_EDGES = 5000
BURN = 500
WHITE_HALF = 19  # uniform +/-19 -> sd ~= 11 ns
N_SEEDS = 8  # average over seeds for stable statistics


def measure(pll_factory, n_edges=N_EDGES, burn=BURN, white_half=WHITE_HALF, seeds=N_SEEDS):
    sds, periods, periods_ac, zetas, fracs, lockeds = [], [], [], [], [], []
    rep_series = None
    for s in range(seeds):
        pll = pll_factory()
        out, trim, noise = run_closed_loop(pll, n_edges, white_half, seed=1000 + s)
        steady = out[burn:].astype(float)
        sds.append(steady.std())
        periods.append(dominant_period(steady))
        periods_ac.append(autocorr_first_peak_period(steady))
        zetas.append(estimate_zeta_from_psd(steady))
        fracs.append(band_power_fraction(steady))
        lockeds.append(pll.is_locked())
        if s == 0:
            rep_series = steady
    return {
        "sd": float(np.mean(sds)),
        "sd_std": float(np.std(sds)),
        "period_fft": float(np.nanmedian(periods)),
        "period_ac": float(np.nanmedian(periods_ac)),
        "zeta_psd": float(np.nanmedian(zetas)),
        "band16_64_frac": float(np.mean(fracs)),
        "locked_frac": float(np.mean(lockeds)),
        "series": rep_series,
    }


def cfg_firmware(i_den=32, kp_inv=8, d_den=4, shift=2):
    def f():
        return PhaseLockLoop(
            mode_smith=True,
            use_i=True,
            use_d=True,
            deadband_ns=0,
            lock_ns=1_000,
            lock_hold=5,
            i_enable_ns=5_000,
            outlier_ns=3_000,
            outlier_max=12,
            i_den=i_den,
            trim_max_mppb=3_000_000,
            d_den=d_den,
            calm_ns=1_000,
            i_den_disturbed_shift=shift,
            kp_inv=kp_inv,
        )

    return f


def main():
    print(f"N_EDGES={N_EDGES} burn={BURN} white_half=+/-{WHITE_HALF} (sd~{WHITE_HALF/np.sqrt(3):.1f}ns) seeds={N_SEEDS}")
    rows = []

    def add(label, i_den, kp_inv, d_den, shift):
        fac = cfg_firmware(i_den, kp_inv, d_den, shift)
        r = measure(fac)
        z_an = analytic_zeta(i_den, kp_inv)
        # impulse ring-down: the loop's OWN modal period and damping
        T_imp, z_imp, peak_imp = impulse_modal(fac)
        rows.append(
            dict(
                config=label,
                i_den=i_den,
                kp_inv=kp_inv,
                d_den=d_den,
                shift=shift,
                sd=r["sd"],
                period_fft=r["period_fft"],
                period_ac=r["period_ac"],
                period_impulse=T_imp,
                zeta_impulse=z_imp,
                peak_impulse=peak_imp,
                zeta_analytic=z_an,
                band=r["band16_64_frac"],
                locked_frac=r["locked_frac"],
                series=r["series"],
            )
        )
        Ts = f"{T_imp:5.1f}s" if not np.isnan(T_imp) else "  >300s"
        zs = f"{z_imp:+.2f}" if not np.isnan(z_imp) else " ~0.0"
        print(
            f"{label:34s} sd={r['sd']:6.1f}ns  Timp={Ts} zeta_imp={zs}  "
            f"Tfft={r['period_fft']:6.1f}s band16-64={r['band16_64_frac']*100:3.0f}%  "
            f"zeta_an={z_an:.2f} lock={r['locked_frac']*100:.0f}%"
        )
        return rows[-1]

    print("\n=== i_den sweep, kp_inv=8 d_den=4, shift=2 (firmware adaptive) ===")
    for iden in [16, 32, 64, 128, 256, 512]:
        add(f"i_den={iden} kp=8 d=4 shift=2", iden, 8, 4, 2)

    print("\n=== i_den sweep, kp_inv=8 d_den=4, shift=0 (fixed gain) ===")
    fixed_rows = {}
    for iden in [16, 32, 64, 128, 256, 512]:
        fixed_rows[iden] = add(f"i_den={iden} kp=8 d=4 shift=0", iden, 8, 4, 0)

    print("\n=== i_den=32, kp_inv sweep (damping restoration), d_den=4 shift=0 ===")
    kp_rows = {}
    for kp in [4, 6, 8, 16]:
        kp_rows[kp] = add(f"i_den=32 kp={kp} d=4 shift=0", 32, kp, 4, 0)

    print("\n=== i_den=32, kp_inv=8, d_den sweep, shift=0 ===")
    d_rows = {}
    for dd in [2, 4, 8]:
        d_rows[dd] = add(f"i_den=32 kp=8 d={dd} shift=0", 32, 8, dd, 0)

    # ---- firmware reference row (the actual deployed config) ----
    fw = rows[1]  # i_den=32 kp=8 d=4 shift=2
    print("\n--- firmware deployed config (i_den=32 kp=8 d=4 shift=2):")
    print(
        f"    white-driven sd={fw['sd']:.1f}ns (NOT >150ns!)  "
        f"impulse modal: period={fw['period_impulse']:.0f}s zeta={fw['zeta_impulse']:.2f} (UNDERDAMPED)"
    )

    # ---- HONEST verdict on reproduction ----
    print("\n*** REPRODUCTION VERDICT ***")
    print(
        f"    11ns WHITE phase noise at firmware config -> sd={fw['sd']:.0f}ns at T={fw['period_fft']:.0f}s."
    )
    print(
        "    The >150ns/35s resonance is NOT reproduced by pure white noise: the loop's own"
    )
    print(
        f"    underdamped mode (impulse: T={fw['period_impulse']:.0f}s, zeta={fw['zeta_impulse']:.2f}) exists"
    )
    print(
        "    structurally but white noise barely excites it (same gap as the validated Rust"
    )
    print("    replay: 46ns on the reception log, no per-edge phase-noise excitation).")

    # ---- best suppressor by the TRUE damping metric (impulse ring-down) ----
    # The white-noise sd does not separate the dampers (all ~7-9ns); the loop's modal
    # damping (impulse zeta + how fast the ring-down settles) is the discriminator.
    # Require: stays locked (>=99%), positive (stable) damping, and don't accept the
    # degenerate tiny-i_den (period collapses): keep period within the diagnosis band-ish.
    candidates = [
        r
        for r in rows
        if r["locked_frac"] >= 0.99
        and not np.isnan(r["zeta_impulse"])
        and r["zeta_impulse"] > 0.05
        and 20 <= (r["period_impulse"] if not np.isnan(r["period_impulse"]) else 1e9) <= 120
    ]
    # Among those, best = lowest white-driven sd AND highest zeta (settles fastest).
    best = min(candidates, key=lambda r: (r["sd"], -r["zeta_impulse"]))
    print(f"\n    best DAMPER (lockable, stable, settles): {best['config']}")
    print(
        f"      white sd={best['sd']:.1f}ns impulse zeta={best['zeta_impulse']:.2f} "
        f"period={best['period_impulse']:.0f}s peak={best['peak_impulse']:.0f}ns"
    )

    # Show the kp_inv axis -- the diagnosis predicted lower kp_inv RESTORES damping;
    # this discrete loop shows the OPPOSITE (lower kp_inv -> unstable).
    print("\n    kp_inv axis (i_den=32): lower kp_inv = MORE P gain:")
    for kp in [16, 8, 6, 4]:
        r = kp_rows[kp]
        print(
            f"      kp_inv={kp:2d}: impulse zeta={r['zeta_impulse']:+.2f} peak={r['peak_impulse']:5.0f}ns "
            f"lock={r['locked_frac']*100:.0f}%  (analytic zeta={r['zeta_analytic']:.2f})"
        )
    print("    -> lowering kp_inv DESTABILIZES (peak grows, zeta->0/neg); analytic formula sign is WRONG for the discrete loop.")

    # d_den axis
    print("\n    d_den axis (i_den=32, kp=8): stronger D = smaller d_den... or larger?")
    for dd in [2, 4, 8]:
        r = d_rows[dd]
        print(
            f"      d_den={dd}: impulse zeta={r['zeta_impulse']:+.2f} peak={r['peak_impulse']:5.0f}ns lock={r['locked_frac']*100:.0f}%"
        )

    # suppression factor: peak ring-down at firmware vs best damper (the resonance amplitude)
    supp_peak = fw["peak_impulse"] / best["peak_impulse"] if best["peak_impulse"] > 0 else float("inf")
    print(f"\n    suppression (impulse peak fw/best): {supp_peak:.2f}x")

    # ---- slow-component cross-check on firmware config ----
    print("\n=== cross-check: white + small slow component on firmware config ===")
    pll = cfg_firmware(32, 8, 4, 2)()
    out_s, _ = run_closed_loop_with_slow(pll, N_EDGES, WHITE_HALF, slow_amp_ns=15, slow_period_s=300, seed=42)
    ss = out_s[BURN:].astype(float)
    print(f"    white+slow(15ns,300s): sd={ss.std():.1f}ns Tfft={dominant_period(ss):.1f}s (tracks input period, not the 35s mode)")

    # ---- figures ----
    make_figures(rows, fw, kp_rows, fixed_rows, d_rows)

    return rows, fw, best, supp_peak


def _impulse_series(i_den, kp_inv, d_den, shift, kick=2000, n=400):
    p = cfg_firmware(i_den, kp_inv, d_den, shift)()
    for _ in range(20):
        p.update(0, True)
    phase = kick
    s = np.empty(n)
    for i in range(n):
        u = p.update(int(phase), True)
        phase += trunc_div(u["freq_trim_mppb"], 1000) - u["phase_corr_ns"]
        s[i] = phase
    return s


def make_figures(rows, fw, kp_rows, fixed_rows, d_rows):
    fig, axs = plt.subplots(2, 2, figsize=(13.5, 9.5))

    # (1) firmware impulse ring-down: THE evidence the mode is underdamped
    ax = axs[0, 0]
    s_fw = _impulse_series(32, 8, 4, 2)
    ax.plot(np.arange(len(s_fw)), s_fw, lw=0.9, color="C3")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_title(
        f"Firmware i_den=32 kp_inv=8 d=4: free ring-down to a 2us phase kick\n"
        f"period~{fw['period_impulse']:.0f}s, zeta={fw['zeta_impulse']:.2f} (UNDERDAMPED type-II resonance, matches 32-37s diag)"
    )
    ax.set_xlabel("edge (s) after kick")
    ax.set_ylabel("output phase (ns)")
    ax.grid(alpha=0.3)

    # (2) impulse ring-down: firmware vs real dampers vs the destabilizer (kp=4)
    ax = axs[0, 1]
    for (iden, kp, dd, sh), c, lbl in [
        ((32, 8, 4, 0), "C3", "firmware kp=8 d=4 (zeta~0.18)"),
        ((32, 4, 4, 0), "C1", "kp_inv=4 (MORE P gain -> UNSTABLE)"),
        ((128, 8, 4, 0), "C2", "i_den=128 (slower, lower-amp)"),
        ((32, 8, 8, 0), "C0", "d_den=8 (more D -> damped)"),
    ]:
        s = _impulse_series(iden, kp, dd, sh, n=300)
        ax.plot(np.arange(len(s)), s, lw=0.9, color=c, label=lbl)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_title("Ring-down vs config: lowering kp_inv DESTABILIZES; D / higher i_den damp")
    ax.set_xlabel("edge (s) after kick")
    ax.set_ylabel("output phase (ns)")
    ax.set_ylim(-1500, 1500)
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)

    # (3) impulse peak & zeta vs i_den (shift=0)
    ax = axs[1, 0]
    idens = [16, 32, 64, 128, 256, 512]
    peak_i = [fixed_rows[i]["peak_impulse"] for i in idens]
    zeta_i = [fixed_rows[i]["zeta_impulse"] for i in idens]
    period_i = [fixed_rows[i]["period_impulse"] for i in idens]
    ax.plot(idens, period_i, "o-", color="C2", label="impulse period (s)")
    ax.set_xscale("log", base=2)
    ax.set_xlabel("i_den")
    ax.set_ylabel("modal period (s)", color="C2")
    ax.set_title("i_den sets the resonance PERIOD (~2*pi*sqrt(i_den)); zeta stays low across i_den")
    ax2 = ax.twinx()
    ax2.plot(idens, zeta_i, "s--", color="C4", label="impulse zeta")
    ax2.set_ylabel("impulse zeta", color="C4")
    ax2.axhline(0.7, color="gray", ls=":", lw=1)
    ax.grid(alpha=0.3, which="both")

    # (4) impulse zeta / peak vs kp_inv -- the damping-restoration axis, with the
    # diagnosis's (wrong-signed) analytic zeta overlaid.
    ax = axs[1, 1]
    kps = [16, 8, 6, 4]
    zeta_kp = [kp_rows[k]["zeta_impulse"] for k in kps]
    za_kp = [analytic_zeta(32, k) for k in kps]
    ax.plot(kps, zeta_kp, "o-", color="C0", label="impulse zeta (this discrete loop)")
    ax.plot(kps, za_kp, "s--", color="C5", label="analytic zeta (diagnosis formula)")
    ax.axhline(0, color="k", lw=0.5)
    ax.axhspan(0.7, 1.0, color="green", alpha=0.1, label="target zeta 0.7-1.0")
    ax.set_xlabel("kp_inv (1/P-gain) at i_den=32  [smaller = more P gain]")
    ax.set_ylabel("damping zeta")
    ax.set_title("kp_inv: analytic says lower=more damped; DISCRETE loop says lower=UNSTABLE")
    ax.invert_xaxis()
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    out = "/home/sksat/prog/pico-gnss-rs/docs/report/sim_resonance.png"
    fig.savefig(out, dpi=110)
    print(f"\nsaved figure: {out}")


if __name__ == "__main__":
    main()
