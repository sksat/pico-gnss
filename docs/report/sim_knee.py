# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "scipy", "matplotlib"]
# ///
"""Locate the knee: where FF-ramp leakage (grows ~linearly with i_den) overtakes
the HW-anchored mode wander (drops ~i_den^-1.28). Report crossover i_den for a
few plausible crystal-drift accelerations."""
import sys, numpy as np
sys.path.insert(0, "/home/sksat/prog/pico-gnss-rs/report")
from sim_iden_sweep_ff import run_plant

HW = {32: 660.0, 128: 112.0}
p = np.log(HW[128] / HW[32]) / np.log(128 / 32)
C = HW[32] / (32.0 ** p)
def mode(i): return C * i ** p

# fit FF-leak as a function of i_den at a reference accel, then scale linearly in accel.
# measure slope k(ff) = leak/i_den in the linear regime.
print("knee analysis: mode wander (HW power-law) vs FF-ramp leak")
print(f"mode(i_den) = {C:.0f} * i_den^{p:.2f}")
print()
idens = [128, 256, 512, 1024, 2048]
for ff in [0.005, 0.01, 0.02, 0.05]:
    leaks = {}
    for i in idens:
        o, _ = run_plant(i, np.zeros(14000, dtype=np.int64), ff_freq_accel_ns_per_s2=ff)
        leaks[i] = np.mean(np.abs(o[7000:]))
    # crossover: smallest i_den where leak >= mode
    cross = None
    for i in idens:
        if leaks[i] >= mode(i):
            cross = i
            break
    total = {i: np.sqrt(mode(i)**2 + leaks[i]**2) for i in idens}
    best = min(total, key=total.get)
    print(f"ff={ff:5.3f} ns/s/s | "
          + " ".join(f"i{i}:m{mode(i):.0f}/l{leaks[i]:.0f}/t{total[i]:.0f}" for i in idens)
          + f" | knee~{cross} best_total@{best}({total[best]:.0f}ns)")
print("\n(m=mode, l=FF-leak, t=quad-sum total. 'best_total' = i_den minimizing total wander)")
print("ff=0.005 is ~realistic bare crystal (few-ppb warming over minutes); 0.05 is a stress case.")
