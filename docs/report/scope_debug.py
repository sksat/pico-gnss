#!/usr/bin/env python3
"""Scope health/locate diagnostic: where are the CH1(GPS)/CH2(GP4) edges, and is the scope
capturing valid edges? Screenshots the display and scans timebases to find the output-vs-GPS
offset so the capture window can be set right. Read-only on the device; only changes scope view.

  RIGOL_HOST=<ip> python3 docs/report/scope_debug.py
"""
import os, sys, statistics
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from scope_pps import Rigol, rising_edge

os.environ.setdefault("RIGOL_HOST", "192.168.0.11")


def swing(w):
    return max(w) - min(w) if w else 0


with Rigol() as s:
    s.drain_errors()
    for c in (":CHANnel1:DISPlay 1", ":CHANnel2:DISPlay 1",
              ":TRIGger:EDGE:SOURce CHANnel1", ":TRIGger:EDGE:SLOPe POSitive",
              ":TRIGger:EDGE:LEVel 1.65", ":TIMebase:MAIN:OFFSet 0"):
        s.send(c)
    print("probe CH1:", s.query(":CHANnel1:PROBe?"), " CH2:", s.query(":CHANnel2:PROBe?"))
    print("scale CH1:", s.query(":CHANnel1:SCALe?"), " CH2:", s.query(":CHANnel2:SCALe?"),
          " offs CH1:", s.query(":CHANnel1:OFFSet?"), " CH2:", s.query(":CHANnel2:OFFSet?"))
    # locate the edges across widening windows
    for tb in (1e-6, 5e-6, 2e-5, 1e-4, 5e-4):
        s.send(f":TIMebase:MAIN:SCALe {tb}")
        xinc = float(s.query(":WAVeform:XINCrement?"))
        offs, sw1, sw2, npts = [], [], [], 0
        for _ in range(8):
            if not s.single():
                continue
            w1, w2 = s.waveform(1), s.waveform(2)
            npts = len(w2)
            sw1.append(swing(w1)); sw2.append(swing(w2))
            i1, i2 = rising_edge(w1), rising_edge(w2)
            if i1 is not None and i2 is not None:
                offs.append((i2 - i1) * xinc)
        m = f"{statistics.mean(offs)*1e9:.0f}ns (sd {statistics.pstdev(offs)*1e9:.0f})" if len(offs) > 1 else (f"{offs[0]*1e9:.0f}ns" if offs else "—")
        print(f"tb={tb*1e6:6.0f}us/div win=+/-{tb*5e6:.0f}us xinc={xinc*1e9:5.0f}ns npts={npts} "
              f"edges_ok={len(offs)}/8 CH1swing~{int(statistics.mean(sw1)) if sw1 else 0} "
              f"CH2swing~{int(statistics.mean(sw2)) if sw2 else 0} offset={m}")
    s.send(":TIMebase:MAIN:SCALe 1e-6")
    n = s.screenshot("docs/report/scope-debug-iden128.png")
    print(f"screenshot docs/report/scope-debug-iden128.png ({n} bytes)")
    print("errors:", s.drain_errors())
