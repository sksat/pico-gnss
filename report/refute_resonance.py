# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "scipy", "matplotlib"]
# ///
"""Adversarial refutation of the underdamped-resonance diagnosis.

Tests the ALTERNATIVE mechanisms (a)-(e). Reimplements the EXACT pll.rs
PhaseLockLoop::update (integer, truncate-toward-zero) and the thermal_plant.rs
simulate() plant, reduced to constant-crystal: phase += freq_trim_mppb//1000 - phase_corr.

The Python port is cross-checked against the real Rust update() by a separate probe.
NMEA coordinates are never read.
"""
import numpy as np


def idiv(a, b):
    # Rust integer division truncates toward zero
    q = abs(a) // abs(b)
    return q if (a < 0) == (b < 0) else -q


class PLL:
    """Exact port of gnssdo::PhaseLockLoop::update (PidSmith branch parametrized)."""

    def __init__(self, i_den=32, kp_inv=8, d_den=4, calm_ns=1000, shift=2,
                 lock_ns=1000, lock_hold=5, i_enable_ns=5000, outlier_ns=3000,
                 outlier_max=12, deadband=0, trim_max=3_000_000, smith=True, use_i=True, use_d=True):
        self.i_den = i_den; self.kp_inv = kp_inv; self.d_den = d_den
        self.calm_ns = calm_ns; self.shift = shift; self.lock_ns = lock_ns
        self.lock_hold = lock_hold; self.i_enable_ns = i_enable_ns
        self.outlier_ns = outlier_ns; self.outlier_max = outlier_max
        self.deadband = deadband; self.trim_max = trim_max
        self.smith = smith; self.use_i = use_i; self.use_d = use_d
        self.lock_cnt = 0; self.reject_cnt = 0; self.trim = 0
        self.last_pred = 0; self.last_pd = 0

    def update(self, phase_err, valid=True):
        ctrl = phase_err
        pred = ctrl - self.last_pd if self.smith else ctrl
        locked = self.lock_cnt >= self.lock_hold
        p_corr = 0; d_corr = 0; applied = False; rejected = False
        if valid:
            if locked and abs(ctrl) > self.outlier_ns and self.reject_cnt < self.outlier_max:
                self.reject_cnt += 1; rejected = True
            else:
                self.reject_cnt = 0; applied = True
                if self.use_i:
                    if abs(ctrl) < self.i_enable_ns:
                        i_den_eff = self.i_den if abs(pred) < self.calm_ns else (self.i_den << min(self.shift, 20))
                        self.trim = max(-self.trim_max, min(self.trim_max, self.trim - idiv(pred * 1000, i_den_eff)))
                else:
                    self.trim = 0
                if abs(pred) > self.deadband:
                    p_corr = max(-100_000_000, min(100_000_000, idiv(pred, self.kp_inv)))
                if self.use_d and locked:
                    d_corr = max(-100_000_000, min(100_000_000, idiv(pred - self.last_pred, self.d_den)))
                self.lock_cnt = min(self.lock_cnt + 1, self.lock_hold) if abs(ctrl) < self.lock_ns else 0
        if applied:
            self.last_pred = pred
            self.last_pd = p_corr + d_corr
        return dict(trim=self.trim, phase_corr=p_corr + d_corr, pred=pred,
                    applied=applied, locked=locked, p=p_corr, d=d_corr)


def lcg_white(n, amp, seed=1):
    """Flat-spectrum integer noise in [-amp, amp]."""
    s = seed & 0xFFFFFFFF
    out = np.empty(n, dtype=np.int64)
    for i in range(n):
        s = (1103515245 * s + 12345) & 0x7FFFFFFF
        out[i] = (s % (2 * amp + 1)) - amp
    return out


def run_constant_crystal(pll, noise, n=5000, drop=500, freq_trim_extra=0):
    """phase += trim//1000 - phase_corr; controller sees phase+noise. Constant crystal cancelled."""
    phase = 0
    phases = []
    for k in range(n):
        meas = phase + int(noise[k])
        u = pll.update(meas, True)
        phase += idiv(u["trim"], 1000) - u["phase_corr"]
        phases.append(phase)
    return np.array(phases[drop:], dtype=float)


def dominant_period(y, fs=1.0):
    y = y - y.mean()
    if len(y) < 64:
        return 0.0
    f = np.fft.rfftfreq(len(y), d=1 / fs)
    p = np.abs(np.fft.rfft(y * np.hanning(len(y)))) ** 2
    p[0] = 0
    k = np.argmax(p)
    return 1.0 / f[k] if f[k] > 0 else 0.0


def impulse_period_zeta(pll_factory, kick=2000, n=400):
    """Free ring-down to a phase kick; estimate modal period and zeta by log-decrement."""
    pll = pll_factory()
    # settle at lock first
    for _ in range(40):
        pll.update(0, True)
    phase = kick
    series = []
    for _ in range(n):
        meas = phase
        u = pll.update(meas, True)
        phase += idiv(u["trim"], 1000) - u["phase_corr"]
        series.append(phase)
    s = np.array(series, float)
    # find zero crossings to get period; peaks for log-decrement
    zc = np.where(np.diff(np.sign(s)) != 0)[0]
    period = np.nan
    if len(zc) >= 3:
        period = 2 * np.mean(np.diff(zc))  # half-period between crossings
    # extrema magnitudes (alternating) for log decrement
    peaks = []
    for i in range(1, len(s) - 1):
        if (s[i] - s[i - 1]) * (s[i + 1] - s[i]) < 0:
            peaks.append(abs(s[i]))
    zeta = np.nan
    if len(peaks) >= 3:
        a = [p for p in peaks if p > 1e-6]
        if len(a) >= 3:
            # log decrement over successive same-side extrema (every 2nd extremum)
            same = a[::2]
            if len(same) >= 2 and same[0] > 0 and same[1] > 0 and same[1] < same[0]:
                delta = np.log(same[0] / same[1])
                zeta = delta / np.sqrt(4 * np.pi ** 2 + delta ** 2)
    return period, zeta


print("=" * 70)
print("CROSS-CHECK vs Rust will be done separately. Sim configs below.")
print("=" * 70)

# ----- (a) NONLINEAR LIMIT CYCLE from calm_ns crossing -----
# Does FIXED i_den=32 (shift=0) differ from adaptive (shift=2) under white noise?
# And do we EVER see |pred| cross calm_ns=1000 under realistic excitation?
print("\n### (a) calm_ns switching limit-cycle test")
amp = 19  # sd ~11ns
for shift in (0, 2):
    pll = PLL(i_den=32, kp_inv=8, d_den=4, shift=shift, calm_ns=1000)
    noise = lcg_white(5000, amp, seed=7)
    y = run_constant_crystal(pll, noise)
    # how often did |pred| exceed calm_ns during the run?
    pll2 = PLL(i_den=32, kp_inv=8, d_den=4, shift=shift, calm_ns=1000)
    noise2 = lcg_white(5000, amp, seed=7)
    phase = 0; crossings = 0; preds = []
    for k in range(5000):
        u = pll2.update(phase + int(noise2[k]), True)
        preds.append(u["pred"])
        if abs(u["pred"]) > 1000:
            crossings += 1
        phase += idiv(u["trim"], 1000) - u["phase_corr"]
    print(f"  shift={shift}: wander_sd={y.std():.1f}ns dom_period={dominant_period(y):.1f}s "
          f"|pred|>calm_ns count={crossings}/5000  max|pred|={max(abs(p) for p in preds)}ns")

# Now drive the loop HARD enough that |pred| oscillates across +-1000 deliberately:
# inject a slow sinusoid disturbance that makes phase swing ~+-1500ns and see if the
# adaptive switching creates a sustained limit cycle that fixed-gain does NOT.
print("\n  -- forced large-swing (phase driven to ~+-1500ns by slow sine) --")
for shift in (0, 2):
    pll = PLL(i_den=32, kp_inv=8, d_den=4, shift=shift, calm_ns=1000)
    phase = 0; series = []
    n = 6000
    drive = 1500 * np.sin(2 * np.pi * np.arange(n) / 200.0)  # slow 200s sine in the MEASUREMENT
    noise = lcg_white(n, amp, seed=3)
    for k in range(n):
        meas = phase + int(drive[k]) + int(noise[k])
        u = pll.update(meas, True)
        phase += idiv(u["trim"], 1000) - u["phase_corr"]
        series.append(phase)
    s = np.array(series[1000:], float)
    print(f"  shift={shift}: residual_sd(after removing drive band)={s.std():.1f}ns dom_period={dominant_period(s):.1f}s")

# ----- (d) period-vs-i_den scaling (IMPULSE modal period) -----
print("\n### (d) modal period vs i_den  (2*pi*sqrt(i_den) prediction)")
for iden in (16, 20, 24, 32, 64, 128):
    per, zeta = impulse_period_zeta(lambda iden=iden: PLL(i_den=iden, kp_inv=8, d_den=4, shift=0, calm_ns=1000))
    pred_per = 2 * np.pi * np.sqrt(iden)
    print(f"  i_den={iden:3d}: impulse_period={per:6.1f}s  2pi*sqrt={pred_per:5.1f}s  zeta={zeta:.3f}")

# ----- (c) quantization-driven limit cycle -----
# Add 16ns capture quantization + period-word freq quantization to see if a limit cycle
# sustains with NO white noise at all (pure quantization).
print("\n### (c) quantization-only limit cycle (no white noise)")
def run_quantized(pll, n=5000, drop=500, cap_q=16, freq_q_mppb=0, kick=0):
    phase = kick
    phases = []
    for k in range(n):
        meas = (phase // cap_q) * cap_q if cap_q > 1 else phase  # 16ns capture quantization
        u = pll.update(int(meas), True)
        trim = u["trim"]
        if freq_q_mppb > 0:
            trim = (trim // freq_q_mppb) * freq_q_mppb  # period-word freq quantization
        phase += idiv(trim, 1000) - u["phase_corr"]
        phases.append(phase)
    return np.array(phases[drop:], float)

for cap_q, fq in [(16, 0), (16, 1000), (1, 1000), (16, 0)]:
    pll = PLL(i_den=32, kp_inv=8, d_den=4, shift=0, calm_ns=1000)
    y = run_quantized(pll, cap_q=cap_q, freq_q_mppb=fq, kick=0)
    print(f"  cap_q={cap_q}ns freq_q={fq}mppb: sd={y.std():.2f}ns dom_period={dominant_period(y):.1f}s "
          f"p2p={y.max()-y.min():.0f}ns")
# with a kick to see if quantization sustains the ring (limit cycle vs decay)
print("  -- with 2us kick, does quantization sustain a limit cycle? --")
for cap_q, fq in [(16, 0), (16, 1000)]:
    pll = PLL(i_den=32, kp_inv=8, d_den=4, shift=0, calm_ns=1000)
    y = run_quantized(pll, n=2000, drop=0, cap_q=cap_q, freq_q_mppb=fq, kick=2000)
    tail = y[-500:]
    print(f"  cap_q={cap_q} freq_q={fq}: tail500 sd={tail.std():.1f}ns p2p={tail.max()-tail.min():.0f}ns "
          f"(sustained if >>0)")
