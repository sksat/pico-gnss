# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "scipy", "matplotlib"]
# ///
"""Is the 35s wander a self-excited underdamped RESONANCE, or a DRIVEN oscillation?

Discriminators:
 1. Coherence/Q: a sharp resonance has a narrow spectral peak (high Q). Estimate the peak's
    fractional bandwidth from the uniform-grid spectrum.
 2. Amplitude vs period: across logs the PERIOD is ~35s regardless of amplitude (100ns..600ns).
    A resonance MODE has fixed period set by i_den; the amplitude is set by the excitation.
    -> if period is constant while amplitude varies 6x, that's a fixed mode driven harder, not
       a nonlinearity whose period would shift with amplitude.
 3. Envelope: bursty (intermittent ring-down after disturbances) vs continuous (steady driven).
 4. Stationarity of the 35s phase: a true ringy mode has slowly-varying phase; broadband would not.

No NMEA coordinates read.
"""
import re
import sys
import numpy as np

LOGS = ["logs/pps-review.log", "logs/pps-slew.log", "logs/pps-proven.log",
        "logs/pps-gain20.log", "logs/pps-iden16.log", "logs/pps-iden32.log"]
TS = re.compile(r"^(\d+\.\d+)\s")


def kv(line, key):
    m = re.search(key + r"=(-?\d+)", line)
    return int(m.group(1)) if m else None


def longest_run(path):
    g = []
    for line in open(path, errors="ignore"):
        m = TS.match(line)
        if not m or "PPSGEN count=" not in line:
            continue
        t = float(m.group(1)); hw = kv(line, "hwphase_ns"); lk = kv(line, "lk")
        if hw is None or lk is None:
            continue
        g.append((t, hw, lk))
    g = np.array(g, float)
    t, hw, lk = g[:, 0], g[:, 1], g[:, 2].astype(int)
    t0 = t[0]
    good = (lk == 1) & (t - t0 > 90) & (np.abs(hw) < 5000)
    idx = np.where(good)[0]
    runs = np.split(idx, np.where(np.diff(idx) != 1)[0] + 1)
    runs = [r for r in runs if len(r) >= 120]
    if not runs:
        return None
    best = max(runs, key=len)
    tr, hr = t[best], hw[best]
    grid = np.arange(tr[0], tr[-1], 1.0)
    hu = np.interp(grid, tr, hr)
    return hu - hu.mean()


for path in LOGS:
    hu = longest_run(path)
    name = path.split("/")[-1]
    if hu is None:
        print(f"{name}: no run"); continue
    n = len(hu)
    w = np.hanning(n)
    sp = np.abs(np.fft.rfft(hu * w)) ** 2
    f = np.fft.rfftfreq(n, 1.0)
    sp[0] = 0
    k = np.argmax(sp)
    pk_per = 1 / f[k]
    # fractional bandwidth (Q): width at half-power around peak
    half = sp[k] / 2
    lo = k
    while lo > 1 and sp[lo] > half:
        lo -= 1
    hi = k
    while hi < len(sp) - 1 and sp[hi] > half:
        hi += 1
    df = f[hi] - f[lo]
    Q = f[k] / df if df > 0 else np.inf
    # fraction of total power in the peak +-1 bin region (16-64s band)
    with np.errstate(divide="ignore"):
        per_all = np.where(f > 0, 1 / np.where(f > 0, f, 1), np.inf)
    bandmask = (per_all >= 16) & (per_all <= 64)
    band = sp[bandmask].sum() / sp[1:].sum() if sp[1:].sum() > 0 else 0
    # envelope burstiness: ratio of p95 of |analytic envelope| to median (bursty if >>1)
    # crude envelope via rolling rms in a ~10s window
    win = 12
    env = np.array([hu[max(0, i - win):i + win].std() for i in range(n)])
    burst = np.percentile(env, 95) / (np.median(env) + 1e-9)
    print(f"{name:20s} sd={hu.std():4.0f}ns peak={pk_per:4.1f}s Q={Q:4.1f} "
          f"band16-64={100*band:3.0f}% burst(p95/med)={burst:.2f}")

print()
print("INTERPRETATION:")
print(" - Q ~ a few (not >>10): a BROAD underdamped peak (low-Q resonance), consistent with zeta~0.2-0.4,")
print("   NOT a razor-sharp driven tone. This is the fingerprint of a lightly-damped MODE rung by noise.")
print(" - period ~35s across sd from ~100 to ~600ns: amplitude-INDEPENDENT period => a fixed linear mode")
print("   (a nonlinear/limit-cycle period would shift with amplitude). Supports 'mode set by i_den'.")
print(" - burst>1 => intermittent ring-downs (disturbance-excited), matching the step/impulse picture,")
print("   NOT steady continuous white drive.")
