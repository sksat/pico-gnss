# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "scipy", "matplotlib"]
# ///
"""Empirical refutation tests on the real logs (no NMEA coordinates read).

(a) Does |hwphase| cluster at the +-1000ns calm_ns switching threshold? (limit-cycle signature)
(b) Is there a coherent ~300s peak (K-recal cadence)? -> check power in 256-512s band & at 306s.
(d) Per-log dominant period vs the loop's actual i_den (32 for all; "gain20" filename).
(e) Aliasing artifact: re-derive the period from AUTOCORRELATION on a UNIFORMLY-resampled,
    gap-free clean-lock run (independent of Lomb-Scargle on irregular samples).
"""
import re
import sys
import numpy as np
from scipy.signal import lombscargle

LOGS = sys.argv[1:] or ["logs/pps-review.log", "logs/pps-slew.log",
                        "logs/pps-proven.log", "logs/pps-gain20.log"]
TS = re.compile(r"^(\d+\.\d+)\s")


def kv(line, key):
    m = re.search(key + r"=(-?\d+)", line)
    return int(m.group(1)) if m else None


def load(path):
    gen = []
    for line in open(path, errors="ignore"):
        m = TS.match(line)
        if not m or "PPSGEN count=" not in line:
            continue
        t = float(m.group(1))
        hw = kv(line, "hwphase_ns")
        lk = kv(line, "lk")
        pred = kv(line, "p_ns")  # p_corr proxy not pred; we will recompute crossing differently
        if hw is None or lk is None:
            continue
        gen.append((t, hw, lk))
    g = np.array(gen, float)
    return g


for path in LOGS:
    g = load(path)
    if len(g) == 0:
        print(f"{path}: no data"); continue
    t, hw, lk = g[:, 0], g[:, 1], g[:, 2].astype(int)
    t0 = t[0]
    good = (lk == 1) & (t - t0 > 90) & (np.abs(hw) < 5000)
    tg, hwg = t[good], hw[good]
    name = path.split("/")[-1]
    print("=" * 64)
    print(f"{name}: locked-clean n={good.sum()} sd={hwg.std():.0f}ns p2p={hwg.max()-hwg.min():.0f}ns")

    # (a) clustering at +-1000ns? Histogram + fraction near +-1000 vs near 0.
    # A limit cycle pinned to the switch would pile probability mass AT the threshold.
    edges = np.array([-2000, -1200, -800, -400, 0, 400, 800, 1200, 2000])
    hist, _ = np.histogram(hwg, bins=np.linspace(-2500, 2500, 26))
    # mass within +-200ns of the +-1000 thresholds vs +-200ns of 0
    near_thr = np.mean((np.abs(np.abs(hwg) - 1000) < 200))
    near_zero = np.mean(np.abs(hwg) < 200)
    # also the controller-relevant: |hwphase|>1000 fraction (would-switch region) -- but pred uses
    # Smith (ctrl-last_pd), much smaller. Report raw |hw|>1000 as an UPPER bound on switching.
    frac_gt1000 = np.mean(np.abs(hwg) > 1000)
    # bimodality: is the distribution bimodal (peaks away from 0) or unimodal at 0?
    from scipy.stats import kurtosis
    print(f"  (a) |hw| within 200ns of +-1000 thr = {100*near_thr:.1f}%  | within 200ns of 0 = {100*near_zero:.1f}%")
    print(f"      frac |hw|>1000ns = {100*frac_gt1000:.1f}%  kurtosis={kurtosis(hwg):.2f} (neg=>bimodal/flat, ~0=>gaussian)")

    # (b) ~300s recal cadence: LS power at 256-512s band and specifically near 306s.
    y = hwg - hwg.mean()
    periods = np.geomspace(4, 1500, 600)
    pw = lombscargle(tg, y, 2 * np.pi / periods, normalize=True)
    band_256_512 = pw[(periods >= 256) & (periods < 512)].sum() / pw.sum()
    # narrow band around 306s
    near306 = pw[(periods >= 280) & (periods < 340)].sum() / pw.sum()
    pk = periods[np.argmax(pw)]
    print(f"  (b) power 256-512s = {100*band_256_512:.1f}%  | near 306s (280-340) = {100*near306:.1f}%  (recal cadence)")

    # (d) dominant period (LS) -- compare to 2pi*sqrt(32)=35.5 and impulse-modal ~25s
    print(f"  (d) LS dominant period = {pk:.0f}s   (2pi*sqrt32=35.5s; impulse-modal~25s)")

    # (e) ALIASING check: pick the LONGEST gap-free clean run, resample to a uniform 1s grid by
    # nearest edge, and compute the AUTOCORRELATION. The first ACF trough/peak gives the period
    # WITHOUT any Lomb-Scargle on irregular sampling. If LS period is a sampling alias, the
    # uniform-grid ACF period will disagree.
    # find longest run of consecutive locked-clean edges with ~1s spacing
    idx = np.where(good)[0]
    # split into contiguous index runs
    runs = np.split(idx, np.where(np.diff(idx) != 1)[0] + 1)
    runs = [r for r in runs if len(r) >= 120]
    best = None
    for r in runs:
        if best is None or len(r) > len(best):
            best = r
    if best is not None and len(best) >= 120:
        tr, hr = t[best], hw[best]
        # uniform 1s grid
        grid = np.arange(tr[0], tr[-1], 1.0)
        hu = np.interp(grid, tr, hr)
        hu = hu - hu.mean()
        ac = np.correlate(hu, hu, "full")[len(hu) - 1:]
        ac = ac / ac[0]
        # first local max after lag 5 = period
        per_acf = None
        for L in range(5, min(200, len(ac) - 1)):
            if ac[L] > ac[L - 1] and ac[L] > ac[L + 1] and ac[L] > 0.05:
                per_acf = L; break
        # first zero crossing (quarter-ish) as sanity
        zc = next((L for L in range(1, len(ac)) if ac[L] < 0), None)
        print(f"  (e) longest gap-free run = {len(best)} edges. UNIFORM-grid ACF: "
              f"first-peak period={per_acf}s  first-zerocross={zc}s "
              f"(4*zc~={4*zc if zc else None}s) -- compare to LS {pk:.0f}s")
    else:
        print(f"  (e) no gap-free run >=120 edges (heavily unlock-gapped); ACF not robust")
