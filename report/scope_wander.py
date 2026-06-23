#!/usr/bin/env python3
"""Robust, SELF-CONFIGURING oscilloscope capture of the GPS->output 1PPS phase wander.

Unlike scope_pps.py `phase` (which only set trigger+timebase and so silently broke when the
front-panel probe ratio didn't match the physical probes), this sets the FULL vertical config
(probe ratio, scale, offset), trigger, timebase and memory depth every run, and records a
WALL-CLOCK TIMESTAMP per shot so the wander spectrum (period) can be computed offline. Use it for
the external before/after that VERIFIES the firmware self-measurement (hwphase) — the scope is for
verify/debug, tuning is on hwphase.

  CH1 = GPS 1PPS (trigger, 1x DIRECT — not a 10x probe), CH2 = disciplined output (GP4 / GP3) via 10x probe.
  NOTE: CH1 is a direct (1x) tap. Forcing CH1 to 10x clips the 3.3V signal off the screen top
  (raw bytes rail -> a bogus ~5.8V readout); the edge timing still works but the voltage is garbage.

  RIGOL_HOST=<ip> python3 report/scope_wander.py <N> <out.log> [tag]

out.log columns:  t_s   offset_ns      (t_s = seconds since capture start)
"""
import os, sys, time, statistics
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from scope_pps import Rigol, rising_edge

N = int(sys.argv[1]) if len(sys.argv) > 1 else 200
OUT = sys.argv[2] if len(sys.argv) > 2 else "logs/scope-wander.log"
TAG = sys.argv[3] if len(sys.argv) > 3 else ""
os.environ.setdefault("RIGOL_HOST", "192.168.0.11")

# Full self-configuration — independent of whatever the front-panel knobs are at.
SETUP = [
    ":CHANnel1:DISPlay 1", ":CHANnel2:DISPlay 1",
    ":CHANnel3:DISPlay 0", ":CHANnel4:DISPlay 0",
    ":CHANnel1:COUPling DC", ":CHANnel2:COUPling DC",
    ":CHANnel1:PROBe 1", ":CHANnel2:PROBe 10",      # CH1 GPS = 1x DIRECT, CH2 output = 10x probe (verified 2026-06)
    ":CHANnel1:SCALe 1.0", ":CHANnel2:SCALe 1.0",   # 1 V/div: 0..3.3 V on-screen (both read ~3.3 V now)
    ":CHANnel1:OFFSet -1.5", ":CHANnel2:OFFSet -1.5",
    ":TRIGger:MODE EDGE", ":TRIGger:EDGE:SOURce CHANnel1",
    ":TRIGger:EDGE:SLOPe POSitive", ":TRIGger:EDGE:LEVel 1.65",
    ":TIMebase:MAIN:OFFSet 0", ":TIMebase:MAIN:SCALe 1e-6",  # 1 us/div, +/-5 us window
    ":ACQuire:MDEPth 10000",                                  # ~1 ns/pt over the 10 us window
]


def main():
    with Rigol() as s:
        s.drain_errors()
        for c in SETUP:
            s.send(c)
        time.sleep(0.3)
        xinc = float(s.query(":WAVeform:XINCrement?"))
        # sanity: confirm both channels actually have a real edge before trusting the run
        s.single()
        w1, w2 = s.waveform(1), s.waveform(2)
        sw1, sw2 = max(w1) - min(w1), max(w2) - min(w2)
        print(f"setup ok: xinc={xinc*1e9:.1f}ns/pt  CH1swing={sw1} CH2swing={sw2}  probe={s.query(':CHAN2:PROBe?')}")
        if sw1 < 100 or sw2 < 100:
            print(f"WARNING: weak edge (CH1={sw1} CH2={sw2}) — check probe/connection; aborting")
            return
        t0 = time.time()
        rows = []
        for _ in range(N):
            if not s.single():
                continue
            w1, w2 = s.waveform(1), s.waveform(2)
            i1, i2 = rising_edge(w1), rising_edge(w2)
            if i1 is None or i2 is None or not (15 < i2 < len(w2) - 15):
                continue
            rows.append((time.time() - t0, (i2 - i1) * xinc * 1e9))
        if len(rows) < 2:
            print(f"FAILED: N_ok={len(rows)}/{N}")
            return
        offs = [r[1] for r in rows]
        m, sd = statistics.mean(offs), statistics.pstdev(offs)
        dur = rows[-1][0]
        print(f"{TAG} N_ok={len(rows)}/{N}  dur={dur:.0f}s  mean={m:.0f}ns  sigma={sd:.0f}ns  "
              f"min={min(offs):.0f} max={max(offs):.0f} pp={max(offs)-min(offs):.0f}ns")
        with open(OUT, "w") as f:
            f.write(f"# scope GPS->output phase wander  tag={TAG}  N_ok={len(rows)}/{N} dur={dur:.0f}s\n")
            f.write("# t_s\toffset_ns\n")
            for t, o in rows:
                f.write(f"{t:.2f}\t{o:.1f}\n")
        print(f"wrote {OUT} ({len(rows)} samples)")


if __name__ == "__main__":
    main()
