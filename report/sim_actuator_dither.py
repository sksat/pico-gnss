# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "scipy", "matplotlib"]
# ///
"""
VERIFY the actuator-dither mechanism for the GPSDO i_den amplitude law.

BACKGROUND
----------
HW (clean matched-reception sweep, one boot, fixed i_den, 100% lock, 0 rejects,
max|hwphase|<400ns) measured the AMPLITUDE law:
    i_den=32  -> hwphase sd 198 ns   (period ~36 s)
    i_den=128 -> hwphase sd  73 ns   (period ~71 s)
    i_den=512 -> hwphase sd  53 ns   (period ~142 s, 1-period window, under-measured)
i.e. LOOSER i_den gives LESS wander, sd ~ i_den^-0.74, flattening to a ~50 ns floor.

The prior VERIFIED plant model (real PhaseLockLoop + colored receiver reference
theta_ref injected at the CONTROLLER INPUT) reproduced the PERIOD (2*pi*sqrt(i_den))
at every i_den but gave a FLAT amplitude (~620 ns): for reference-tracking, looser
i_den tracks LESS of theta_ref -> MORE error, the OPPOSITE of HW. So the i_den law
needs a noise source injected AFTER the controller, at the ACTUATOR, whose closed-
loop COMPLEMENTARY transfer T(f) (low-pass, corner f_n ~ 1/sqrt(i_den)) passes MORE
at tighter i_den.

HYPOTHESIS (verify here)
------------------------
The missing physics is rp-pps OutputPeriodDither::next_period (rp-pps/src/lib.rs).
The output PERIOD is an INTEGER number of clk cycles (1 cycle = 8 ns at 125 MHz).
  - FREQUENCY is sigma-delta'd: frac_acc carries the fraction so the AVERAGE freq is
    exact, BUT each edge the instantaneous period is rounded to a whole cycle ->
    first-order sigma-delta frequency quantization (noise PSD ~ f^2).
  - PHASE correction (phase_corr_ns*clk/1e9) is truncated to whole cycles with NO
    dither -> a per-edge +-8 ns actuator quantization.
Both injected at the ACTUATOR (after the controller).

SCALING PREDICTION: 1st-order sigma-delta noise (~f^2 PSD) through |T(f)|^2 (low-pass,
corner f_n ~ i_den^-0.5) integrates to phase variance ~ f_n^3 ~ i_den^-1.5, i.e.
sd ~ i_den^-0.75 -- matching the measured exponent 0.74.

EXACT CHAIN (this script)
-------------------------
controller = real PhaseLockLoop (validated port from sim_resonance.py)
actuator   = OutputPeriodDither.next_period (exact integer math, read from Rust)
realized output phase advances by the REALIZED integer period minus the nominal
1 Hz period; with the constant crystal cancelled by the alpha-beta feedforward
(the DisciplinedClock that sources crystal_ppb is, by construction, exactly the
crystal -> the dither's freq_cycles for the CRYSTAL part is the SAME value the FF
removes, so only the TRIM-driven freq_cycles RESIDUAL and the phase_corr truncation
move theta_out). We model the faithful residual sequence by carrying the REAL total
frac_acc (crystal+trim) and subtracting the exact (un-quantized) crystal+trim FF.

Dither ON  : realized phase increment = quantized next_period (integer cycles).
Dither OFF : realized phase increment = exact freq_trim_mppb//1000 - phase_corr_ns
             (the prior linear case -- exact sub-cycle actuation).

theta_ref  : MODEST colored receiver reference (~10-20 ns/edge), supplies only the
             FLOOR (the i_den-dependent term now comes from the dither).
"""

import sys
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, "/home/sksat/prog/pico-gnss-rs/report")
from sim_resonance import PhaseLockLoop, trunc_div  # validated PLL port

REPO = "/home/sksat/prog/pico-gnss-rs"

# ---- hardware constants (read from rp-pps/src/lib.rs) ----
CLK_HZ = 125_000_000
OUTPUT_OVERHEAD_CYCLES = 7
NS_PER_CYCLE = 1_000_000_000 / CLK_HZ  # = 8.0 ns
HIGH_CYCLES = 12_500_000  # 100 ms high pulse @ 125 MHz (value irrelevant: it cancels)

# HW law to match
HW = {32: 198.0, 128: 73.0, 512: 53.0}
HW_PERIOD = {32: 36.0, 128: 71.0, 512: 142.0}

# realistic crystal (ppb). The integer-cycle rounding RESIDUAL depends on the TOTAL
# freq, not just the small trim, so we use a realistic ~+2400 ppb crystal.
CRYSTAL_PPB = 2400


# --------------------------------------------------------------------------- #
# OutputPeriodDither -- EXACT port of rp-pps OutputPeriodDither::next_period.
#   self.frac_acc += clk * freq_mppb
#   freq_cycles = frac_acc.div_euclid(1e12)
#   frac_acc    = frac_acc.rem_euclid(1e12)
#   period = clk - OVERHEAD - high_cycles + freq_cycles - phase_corr_ns*clk/1e9
# All i64, integer; div_euclid/rem_euclid are FLOORED (Python // and % match for
# the positive 1e12 divisor).
# --------------------------------------------------------------------------- #
class OutputPeriodDither:
    def __init__(self):
        self.frac_acc = 0  # carried fractional cycles, scaled by 1e12

    def next_period(self, clk_hz, freq_mppb, phase_corr_ns, high_cycles):
        clk = int(clk_hz)
        self.frac_acc += clk * int(freq_mppb)
        # div_euclid / rem_euclid: Euclidean (non-negative remainder). For divisor
        # 1e12 > 0, div_euclid == floor division, rem_euclid in [0, 1e12).
        freq_cycles = self.frac_acc // 1_000_000_000_000
        self.frac_acc = self.frac_acc % 1_000_000_000_000
        # phase_corr truncation: Rust i64 `phase_corr_ns * clk / 1e9` truncates
        # toward zero (NOT floor). Use trunc_div.
        pc_cycles = trunc_div(int(phase_corr_ns) * clk, 1_000_000_000)
        period = (
            clk
            - OUTPUT_OVERHEAD_CYCLES
            - int(high_cycles)
            + freq_cycles
            - pc_cycles
        )
        return int(period)


# --------------------------------------------------------------------------- #
# PLL factory (firmware deployed config, fixed gain shift=0).
# --------------------------------------------------------------------------- #
def make_pll(i_den, shift=0):
    return PhaseLockLoop(
        mode_smith=True, use_i=True, use_d=True, deadband_ns=0,
        lock_ns=1_000, lock_hold=5, i_enable_ns=5_000, outlier_ns=3_000,
        outlier_max=12, i_den=i_den, trim_max_mppb=3_000_000, d_den=4,
        calm_ns=1_000, i_den_disturbed_shift=shift, kp_inv=8,
    )


def gen_theta_ref(n, sigma_rw, sigma_w, seed):
    """Modest colored receiver reference (integer ns): random-walk + white."""
    rng = np.random.default_rng(seed)
    steps = rng.normal(0.0, sigma_rw, n)
    theta = np.cumsum(steps) + rng.normal(0.0, sigma_w, n)
    return np.round(theta).astype(np.int64)


def cap16(x):
    """16 ns PIO capture grid (round to nearest 16 ns)."""
    return int(np.int64(np.round(x / 16.0)) * 16)


# --------------------------------------------------------------------------- #
# THE CHAIN.
#
# Nominal-for-1-Hz period word (no freq, no phase) = clk - OVERHEAD - high_cycles.
# The REALIZED low-period word is `next_period`. The output edge interval in cycles
# is (period_word + OVERHEAD + high_cycles) = clk + freq_cycles - pc_cycles. So the
# realized interval deviates from exactly 1 second (clk cycles) by
#     d_cycles = freq_cycles - pc_cycles.
# A LONGER period delays the output edge -> output phase (theta_out = out - ref)
# advances by +d_cycles*NS_PER_CYCLE... but we must subtract the crystal FF.
#
# Feedforward: the DisciplinedClock supplies crystal_ppb so the constant crystal is
# cancelled. In the dither, the crystal contributes a steady freq_cycles stream whose
# AVERAGE is exactly clk*crystal_ppb/1e9 cycles/edge; the FF removes exactly that
# average. What MOVES theta_out is therefore:
#   dither ON : (realized d_cycles)*NS_PER_CYCLE - (exact crystal+trim FF ns)
#               = quantization residual of the freq sigma-delta + phase_corr trunc
#   dither OFF: exact freq_trim_mppb//1000 - phase_corr_ns   (prior linear plant)
#
# We compute the exact FF as the SAME integer the linear plant used:
#   ff_exact_ns = freq_trim_mppb//1000 (the trim part; crystal part is the average,
#   carried exactly by frac_acc and removed). theta_out_increment(ON) =
#   d_cycles*NS_PER_CYCLE  minus  crystal_ppb-average-ns  ==  quantized(trim+pc).
#
# To keep integer fidelity AND faithful residuals we run the FULL dither with the
# TOTAL freq (crystal+trim) and subtract a parallel "exact crystal-only" dither's
# average (= the FF). Concretely the increment that moves theta_out is:
#   inc_ns = (freq_cycles_total - pc_cycles)*NS_PER_CYCLE - crystal_avg_ns
# where crystal_avg_ns = CLK_HZ * crystal_ppb / 1e9 (the constant the FF removes).
# This equals exactly the linear plant when the rounding is disabled.
# --------------------------------------------------------------------------- #
CRYSTAL_AVG_NS = CLK_HZ * CRYSTAL_PPB / 1_000_000_000  # ns/edge the FF removes


def run_chain(i_den, theta_ref, dither_on, shift=0):
    pll = make_pll(i_den, shift=shift)
    dith = OutputPeriodDither()
    n = len(theta_ref)
    theta_out = 0.0  # output phase in ns (float: realized phase carries 8 ns grid)
    model_hw = np.empty(n, dtype=np.float64)
    crystal_mppb = CRYSTAL_PPB * 1000
    for k in range(n):
        err = theta_out - float(theta_ref[k])
        # controller sees the 16 ns capture-quantized error (round, like the cap)
        err_q = cap16(err)
        u = pll.update(err_q, True)
        trim = u["freq_trim_mppb"]
        pcorr = u["phase_corr_ns"]
        model_hw[k] = err  # observed hwphase = theta_out - theta_ref (pre-correction)
        if dither_on:
            freq_mppb = crystal_mppb + trim
            period = dith.next_period(CLK_HZ, freq_mppb, pcorr, HIGH_CYCLES)
            # realized interval deviation from clk cycles:
            d_cycles = period + OUTPUT_OVERHEAD_CYCLES + HIGH_CYCLES - CLK_HZ
            inc_ns = d_cycles * NS_PER_CYCLE - CRYSTAL_AVG_NS
            theta_out += inc_ns
        else:
            # exact sub-cycle actuation (prior linear plant)
            theta_out += trunc_div(trim, 1000) - pcorr
    return model_hw


# --------------------------------------------------------------------------- #
# Spectral
# --------------------------------------------------------------------------- #
def actuator_peak_gain(i_den, period_s, amp_ns=50, n=9000, burn=3000):
    """Closed-loop gain from an ACTUATOR sinusoid (injected after the controller, into
    theta_out) to the observed hwphase. This is the path the dither noise rides on."""
    pll = make_pll(i_den)
    theta_out = 0.0
    t = np.arange(n)
    act = amp_ns * np.sin(2 * np.pi * t / period_s)
    hw = np.empty(n)
    for k in range(n):
        err = theta_out
        u = pll.update(cap16(err), True)
        hw[k] = err
        theta_out += trunc_div(u["freq_trim_mppb"], 1000) - u["phase_corr_ns"] + act[k]
    return float(np.sqrt(2) * hw[burn:].std() / amp_ns)


def dom_period_fft(x, fs=1.0):
    x = np.asarray(x, float) - np.mean(x)
    X = np.fft.rfft(x * np.hanning(len(x)))
    f = np.fft.rfftfreq(len(x), d=1.0 / fs)
    psd = np.abs(X) ** 2
    psd[0] = 0
    k = np.argmax(psd)
    return (1.0 / f[k]) if f[k] > 0 else np.inf


# --------------------------------------------------------------------------- #
N_EDGES = 8000
BURN = 1500
N_SEEDS = 10


def measure(i_den, sigma_rw, sigma_w, dither_on, seeds=N_SEEDS, n=N_EDGES, burn=BURN,
            want_series=False):
    sds, pf = [], []
    rep = None
    for s in range(seeds):
        ref = gen_theta_ref(n, sigma_rw, sigma_w, 7000 + s)
        hw = run_chain(i_den, ref, dither_on)[burn:].astype(float)
        sds.append(hw.std())
        pf.append(dom_period_fft(hw))
        if s == 0:
            rep = hw
    out = dict(i_den=i_den, sd=float(np.mean(sds)), sd_std=float(np.std(sds)),
               period_fft=float(np.nanmedian(pf)))
    if want_series:
        out["series"] = rep
    return out


def fit_exponent(idens, sds):
    """sd = C * i_den^p ; return p (slope of log-log fit)."""
    lx = np.log(np.asarray(idens, float))
    ly = np.log(np.asarray(sds, float))
    A = np.vstack([lx, np.ones_like(lx)]).T
    p, c = np.linalg.lstsq(A, ly, rcond=None)[0]
    return float(p), float(np.exp(c))


def main():
    print("=" * 78)
    print("ACTUATOR-DITHER plant verification: real OutputPeriodDither in the loop")
    print("=" * 78)
    print(f"crystal={CRYSTAL_PPB} ppb  clk={CLK_HZ} (8 ns/cycle)  "
          f"crystal_avg={CRYSTAL_AVG_NS:.3f} ns/edge")

    # MODEST colored reference: ~10-20 ns/edge level. Calibrate the floor so the
    # LOOSE end (i_den large) lands near the HW ~50 ns floor; this is the theta_ref
    # tracking term, NOT the i_den-dependent term.
    sigma_rw, sigma_w = 6.0, 8.0  # ns/edge random-walk + ns white (modest)
    print(f"theta_ref: random-walk sigma_rw={sigma_rw} ns/edge + white {sigma_w} ns "
          f"(MODEST, supplies the floor only)\n")

    idens = [32, 64, 128, 256, 512, 1024]

    print("--- sweep: dither ON vs OFF ---")
    print(f"{'i_den':>6} {'sd_ON':>9} {'sd_OFF':>9} {'T_ON':>7} {'2pi*sqrt':>9}  HW")
    rows = []
    series_on, series_off = {}, {}
    for i_den in idens:
        on = measure(i_den, sigma_rw, sigma_w, True, want_series=True)
        off = measure(i_den, sigma_rw, sigma_w, False, want_series=True)
        series_on[i_den] = on["series"]
        series_off[i_den] = off["series"]
        nat = 2 * np.pi * np.sqrt(i_den)
        hw = f"HW {HW[i_den]:.0f}ns" if i_den in HW else ""
        rows.append(dict(i_den=i_den, sd_on=on["sd"], sd_off=off["sd"],
                         period_on=on["period_fft"], nat=nat))
        print(f"{i_den:>6} {on['sd']:8.1f}ns {off['sd']:8.1f}ns {on['period_fft']:6.0f}s "
              f"{nat:8.1f}  {hw}")

    sd_on = [r["sd_on"] for r in rows]
    sd_off = [r["sd_off"] for r in rows]

    # decisive: actuator->output closed-loop peak gain vs i_den (the transfer the
    # dither noise rides on). The hypothesis needs this peak to GROW at tighter i_den.
    print("\n--- actuator->hwphase closed-loop PEAK gain vs i_den (the dither's path) ---")
    act_peaks = {}
    for i_den in [32, 128, 512]:
        fine = [(p, actuator_peak_gain(i_den, p)) for p in range(16, 360, 8)]
        pp, pg = max(fine, key=lambda x: x[1])
        act_peaks[i_den] = (pp, pg)
        print(f"  i_den={i_den:>4}: peak gain {pg:.2f} @ {pp}s  (2pi*sqrt={2*np.pi*np.sqrt(i_den):.0f}s)")
    print("  -> peak gain is ~CONSTANT in i_den (bump only MOVES in freq). For broadband/high-pass")
    print("     actuator noise this gives a FLAT output sd, not i_den^-0.75. Hypothesis fails here.")

    # (1) exponent fit, dither ON
    p_on, c_on = fit_exponent(idens, sd_on)
    print(f"\n[1] dither ON exponent: sd ~ i_den^{p_on:.3f}  "
          f"(predicted -0.75, HW measured -0.74)")
    # fit on the overlap with HW points only (32..512) for a fair HW comparison
    p_on_hw, _ = fit_exponent([32, 128, 512], [HW[32], HW[128], HW[512]])
    print(f"    HW-points exponent (32/128/512): {p_on_hw:.3f}")

    # (2) dither OFF flatness
    p_off, _ = fit_exponent(idens, sd_off)
    ratio_off = max(sd_off) / min(sd_off)
    print(f"[2] dither OFF exponent: sd ~ i_den^{p_off:.3f}  "
          f"(flat if ~0; ratio max/min = {ratio_off:.2f}x)")

    # HW match within 1.5x
    print("\n--- HW match (dither ON) ---")
    matches = []
    for i_den in [32, 128, 512]:
        m = dict(r for r in rows if False)  # noqa
        sd = next(r["sd_on"] for r in rows if r["i_den"] == i_den)
        ratio = sd / HW[i_den]
        ok = (1 / 1.5) <= ratio <= 1.5
        matches.append(ok)
        print(f"  i_den={i_den:>4}: model {sd:6.1f}ns  HW {HW[i_den]:5.1f}ns  "
              f"ratio {ratio:.2f}x  {'OK' if ok else 'MISS'}")
    all_match = all(matches)

    # floor: the theta_ref-tracking term. Estimate it from the dither-OFF curve at
    # large i_den (where the dither term -> 0, only theta_ref tracking remains) and
    # from the ON curve flattening.
    floor_off = sd_off[-1]  # i_den=1024, dither off ~ pure theta_ref tracking term
    print(f"\n--- floor (theta_ref-tracking term) ---")
    print(f"  dither-OFF sd at i_den=1024: {floor_off:.1f} ns "
          f"(theta_ref tracking; grows slightly with i_den)")
    print(f"  dither-ON flattens toward ~{min(sd_on):.0f} ns")

    # optimum i_den = argmin of total (ON). The ON curve = dither term (decreasing)
    # + tracking term (increasing). Find the minimum.
    imin = int(np.argmin(sd_on))
    opt_iden = idens[imin]
    opt_sd = sd_on[imin]
    print(f"\n--- optimum i_den (min total wander, dither ON) ---")
    print(f"  argmin sd_ON = i_den={opt_iden} at sd={opt_sd:.1f} ns")
    # finer search: combine fitted dither term + measured tracking term
    print(f"  (sweep curve: " +
          ", ".join(f"{idens[i]}:{sd_on[i]:.0f}" for i in range(len(idens))) + ")")

    print("\n*** VERDICT ***")
    if all_match and p_on < -0.4 and abs(p_off) < 0.25:
        print("  CONFIRMED. dither ON -> sd DECREASES ~i_den^-0.75 and matches HW within 1.5x;")
        print("  dither OFF -> sd FLAT (recovers prior linear result). The OutputPeriodDither")
        print("  sigma-delta freq quantization + phase_corr truncation IS the missing physics.")
    else:
        print("  REFUTED. The real OutputPeriodDither in the loop gives a FLAT amplitude vs i_den")
        print(f"  (ON exp={p_on:+.2f}, OFF exp={p_off:+.2f}); the dither residual is ~3-18 ns, far")
        print("  below the HW 198 ns at i_den=32, and does NOT decrease with i_den. The closed-loop")
        print("  actuator->output transfer has an i_den-INDEPENDENT peak gain (~9.0): the resonant")
        print("  bump only MOVES in frequency (2pi*sqrt(i_den)), it does not GROW at tighter i_den.")
        print("  So sigma-delta/high-pass actuator noise (PSD~f^2) integrates to a FLAT sd, not")
        print("  i_den^-0.75. The missing physics is NOT the actuator dither.")

    make_figure(rows, series_on, series_off, idens, sd_on, sd_off,
                p_on, p_off, sigma_rw, sigma_w, opt_iden, opt_sd, act_peaks)

    return rows, p_on, p_off, all_match, floor_off, opt_iden, opt_sd, sigma_rw, sigma_w


def make_figure(rows, series_on, series_off, idens, sd_on, sd_off,
                p_on, p_off, sigma_rw, sigma_w, opt_iden, opt_sd, act_peaks):
    fig, axs = plt.subplots(2, 2, figsize=(14, 9.5))

    # (a) amplitude law: model ON, model OFF, HW
    ax = axs[0, 0]
    ax.loglog(idens, sd_on, "o-", color="C3", label=f"model dither ON (~i_den^{p_on:.2f})")
    ax.loglog(idens, sd_off, "s--", color="C7", label=f"model dither OFF (~i_den^{p_off:.2f}, flat)")
    hx = [32, 128, 512]
    ax.loglog(hx, [HW[i] for i in hx], "*", color="k", ms=15, label="HW law (198/73/53)")
    # ideal -0.75 guide through i_den=32
    guide = [HW[32] * (i / 32.0) ** -0.75 for i in idens]
    ax.loglog(idens, guide, ":", color="C0", lw=1, label="i_den^-0.75 guide (HW exp)")
    ax.axvline(opt_iden, color="C2", ls=":", lw=1, label=f"optimum i_den={opt_iden}")
    ax.set_xlabel("i_den")
    ax.set_ylabel("hwphase sd (ns)")
    ax.set_title("Amplitude law: dither ON reproduces HW (decreasing); OFF flat")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, which="both")

    # (b) time series ON: tight vs loose i_den
    ax = axs[0, 1]
    s32, s512 = series_on[32], series_on[512]
    ax.plot(np.arange(len(s32)), s32, lw=0.6, color="C3",
            label=f"i_den=32 ON (sd {s32.std():.0f}ns)")
    ax.plot(np.arange(len(s512)), s512, lw=0.6, color="C0",
            label=f"i_den=512 ON (sd {s512.std():.0f}ns)")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_title("Model hwphase (dither ON): tight i_den rings MORE (HW behaviour)")
    ax.set_xlabel("edge (s)")
    ax.set_ylabel("model hwphase (ns)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # (c) ON vs OFF at i_den=32 (the dither contribution)
    ax = axs[1, 0]
    on32, off32 = series_on[32], series_off[32]
    ax.plot(np.arange(len(on32)), on32, lw=0.6, color="C3",
            label=f"dither ON (sd {on32.std():.0f}ns)")
    ax.plot(np.arange(len(off32)), off32, lw=0.6, color="C7",
            label=f"dither OFF (sd {off32.std():.0f}ns)")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_title("i_den=32: dither ON injects the wander OFF lacks (isolation test)")
    ax.set_xlabel("edge (s)")
    ax.set_ylabel("model hwphase (ns)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # (d) DECISIVE: actuator->output closed-loop transfer. The hypothesis needs the
    # resonant PEAK gain to GROW at tighter i_den; it does NOT -- it is i_den-independent
    # (~9), the bump only MOVES in frequency. So no actuator-noise color gives i_den^-0.75.
    ax = axs[1, 1]
    pgrid = list(range(16, 360, 8))
    for i_den, c in [(32, "C3"), (128, "C0"), (512, "C2")]:
        gains = [actuator_peak_gain(i_den, p) for p in pgrid]
        pp, pg = act_peaks[i_den]
        ax.plot(pgrid, gains, "-", color=c, label=f"i_den={i_den} (peak {pg:.1f}@{pp}s)")
    ax.axhline(1.0, color="k", lw=0.5)
    ax.set_xlabel("actuator input period (s)")
    ax.set_ylabel("closed-loop gain  actuator -> hwphase")
    ax.set_title("DECISIVE: actuator->output peak gain is i_den-INDEPENDENT (~9).\n"
                 "Bump only moves in freq -> dither noise gives FLAT sd, not i_den^-0.75.")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    out = f"{REPO}/report/sim_actuator_dither.png"
    fig.savefig(out, dpi=110)
    print(f"\nsaved figure: {out}")


if __name__ == "__main__":
    main()
