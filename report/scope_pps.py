#!/usr/bin/env python3
"""Oscilloscope verification of the disciplined 1PPS output (Rigol DHO800).

An independent cross-check measured straight off the pins, with no firmware in
the loop. Zero dependencies (stdlib socket only).

  CH1 = GPS receiver 1PPS                 (reference / trigger, 1x probe)
  CH2 = RP2040 disciplined 1PPS on GP3    (100 ms pulse, 10x probe)

The 10x ratio on CH2 matters: at 1x a 3.3 V pulse reads 0.33 V and never
crosses a 1.65 V trigger.

  RIGOL_HOST=<scope-ip> python3 scope_pps.py phase [N] [log]  # GPS->gen phase, N shots (+raw log)
  RIGOL_HOST=<scope-ip> python3 scope_pps.py capture [out]    # clean two-trace screenshot

Result on hardware (RP2040 @125MHz, GYSFFMANC, N=60):
  GPS->gen phase: mean=-2803ns  sigma=104.3ns  pp=500ns
  Negative mean = the disciplined output leads GPS (fixed fire/probe offset).
  sigma ~= the receiver's own PPS sawtooth, since the output is PLL-smoothed.

DHO804 (fw 00.01.02) gotchas:
  * The SCPI error queue is FIFO -- drain it fully before blaming a command.
  * :MEASure:CLEar takes NO argument (ALL -> -108 Parameter not allowed).
  * The GPS PPS is a wide (~100 ms) pulse, so a delay/period auto-measurement
    in a us window can't pair edges -- dump the waveform and compute the delta.
"""
import os
import socket
import statistics
import sys
import time

PORT = int(os.environ.get("RIGOL_PORT", "5555"))


class Rigol:
    """Minimal SCPI client for the Rigol DHO800 series over raw TCP."""

    def __init__(self, host=None, port=PORT, timeout=4.0):
        host = host or os.environ.get("RIGOL_HOST")
        if not host:
            raise SystemExit("set RIGOL_HOST=<scope-ip> (Rigol DHO800 SCPI host)")
        self.s = socket.create_connection((host, port), timeout=timeout)
        self.timeout = timeout

    def close(self):
        self.s.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def send(self, cmd):
        self.s.sendall((cmd + "\n").encode())

    def query(self, cmd):
        self.send(cmd)
        self.s.settimeout(self.timeout)
        data = b""
        while not data.endswith(b"\n"):
            chunk = self.s.recv(4096)
            if not chunk:
                break
            data += chunk
        return data.decode(errors="replace").strip()

    def query_block(self, cmd):
        """Read an IEEE-488.2 definite-length block: #<n><len><payload>\\n."""
        self.send(cmd)
        self.s.settimeout(self.timeout)
        buf = b""
        while b"#" not in buf:
            buf += self.s.recv(4096)
        buf = buf[buf.index(b"#"):]
        while len(buf) < 2:
            buf += self.s.recv(64)
        ndigits = int(buf[1:2])
        while len(buf) < 2 + ndigits:
            buf += self.s.recv(64)
        length = int(buf[2:2 + ndigits])
        need = 2 + ndigits + length
        while len(buf) < need:
            buf += self.s.recv(need - len(buf))
        payload = buf[2 + ndigits:need]
        self.s.settimeout(0.25)
        try:
            self.s.recv(2)  # consume trailing newline
        except Exception:
            pass
        self.s.settimeout(self.timeout)
        return payload

    def drain_errors(self):
        out = []
        for _ in range(32):
            e = self.query(":SYSTem:ERRor?")
            if e.startswith("0,") or "No error" in e:
                break
            out.append(e)
        return out

    def waveform(self, ch, mode="NORMal"):
        self.send(f":WAVeform:SOURce CHANnel{ch}")
        self.send(f":WAVeform:MODE {mode}")
        self.send(":WAVeform:FORMat BYTE")
        return self.query_block(":WAVeform:DATA?")

    def single(self, settle=0.05, tries=80):
        self.send(":SINGle")
        for _ in range(tries):
            time.sleep(settle)
            if self.query(":TRIGger:STATus?") == "STOP":
                return True
        return False

    def screenshot(self, path):
        png = self.query_block(":DISPlay:DATA? PNG")
        with open(path, "wb") as f:
            f.write(png)
        return len(png)


def rising_edge(samples, min_swing=25):
    """Sub-sample index of the first rising mid-level crossing, or None."""
    lo, hi = min(samples), max(samples)
    if hi - lo < min_swing:
        return None
    thr = (lo + hi) / 2
    for i in range(1, len(samples)):
        if samples[i - 1] < thr <= samples[i]:
            return (i - 1) + (thr - samples[i - 1]) / (samples[i] - samples[i - 1])
    return None


def cmd_phase(scope, n, logpath=None):
    """Trigger on the GPS edge, dump both channels, report the delta over N shots.

    If logpath is given, write one GPS->gen delta (ns) per line for plotting.
    """
    for c in (
        ":TRIGger:EDGE:SOURce CHANnel1",
        ":TRIGger:EDGE:SLOPe POSitive",
        ":TRIGger:EDGE:LEVel 1.65",
        ":TIMebase:MAIN:SCALe 1e-6",
        ":TIMebase:MAIN:OFFSet 0",
    ):
        scope.send(c)
    xinc = float(scope.query(":WAVeform:XINCrement?"))

    deltas = []
    for _ in range(n):
        if not scope.single():
            continue
        w1 = scope.waveform(1)
        w2 = scope.waveform(2)
        i1 = rising_edge(w1)
        i2 = rising_edge(w2)
        if i1 is None or i2 is None:
            continue
        # Reject window-edge detections: if the gen edge drifted near the screen edge the
        # crossing index is unreliable (or it's noise), so drop that shot rather than log a spike.
        if not (15 < i2 < len(w2) - 15):
            continue
        deltas.append((i2 - i1) * xinc)

    print(f"xinc={xinc * 1e9:.2f}ns/pt  N_ok={len(deltas)}/{n}")
    if len(deltas) > 1:
        m = statistics.mean(deltas)
        sd = statistics.pstdev(deltas)
        print(
            f"GPS->gen phase: mean={m * 1e9:.0f}ns  sigma={sd * 1e9:.1f}ns  "
            f"min={min(deltas) * 1e9:.0f}  max={max(deltas) * 1e9:.0f}  "
            f"pp={(max(deltas) - min(deltas)) * 1e9:.0f}ns"
        )
    if logpath:
        with open(logpath, "w") as f:
            f.write("# GPS->gen 1PPS phase delta [ns], one rising-edge pair per single-shot\n")
            for d in deltas:
                f.write(f"{d * 1e9:.1f}\n")
        print(f"wrote {logpath} ({len(deltas)} samples)")


# Display setup for a presentable capture: CH1 top, CH2 bottom, edges centered.
CAPTURE_SETUP = [
    ":CHANnel1:DISPlay 1", ":CHANnel2:DISPlay 1",
    ":CHANnel3:DISPlay 0", ":CHANnel4:DISPlay 0",
    ":CHANnel1:PROBe 1", ":CHANnel2:PROBe 10",          # GPS direct; gen via 10x probe
    ":CHANnel1:SCALe 0.6", ":CHANnel2:SCALe 0.6",       # 0.6 V/div: 0->3.3 V fills screen, ~2 div top headroom
    ":CHANnel1:OFFSet -2.1", ":CHANnel2:OFFSet -2.1",   # 0 V near bottom; 3.3 V sits ~+2 div, clear of the T marker
    ":TRIGger:EDGE:SOURce CHANnel1",                    # trigger on the GPS (reference) edge...
    ":TRIGger:EDGE:SLOPe POSitive",
    ":TRIGger:EDGE:LEVel 1.65",
    ":TIMebase:MAIN:SCALe 1e-6",                        # 1 us/div
    ":TIMebase:MAIN:OFFSet 0",                          # ...at screen center = 0; gen leads -> to its left
    ":MEASure:STATistic:DISPlay OFF",
    ":MEASure:CLEar",                                   # NOTE: no argument on DHO800
]


def cmd_capture(scope, out, timebase_s=None):
    for c in CAPTURE_SETUP:
        scope.send(c)
    if timebase_s:  # override the default 1 us/div (tighten once the offset is sub-us)
        scope.send(f":TIMebase:MAIN:SCALe {timebase_s}")
    errs = scope.drain_errors()
    if errs:
        print("scope errors:", errs)
    if not scope.single():
        print("warning: no trigger (is the GPS PPS present on CH1?)")
    print(f"wrote {out} ({scope.screenshot(out)} bytes)")


def cmd_converge(scope, out, n=220, timebase_s=1e-6):
    """Log the scope GPS->gen offset vs elapsed time during lock acquisition.

    1 us/div (+-5 us, 10 ns resolution) captures the *detailed* final pull-in into the lock window;
    while the edge is still far out it is off-window and logged as nan (the firmware self-report
    covers that wide early part). Run this while the firmware is freshly booting (alongside
    `cargo run`) to capture the convergence.
    """
    for c in (
        ":CHANnel1:DISPlay 1", ":CHANnel2:DISPlay 1",
        ":CHANnel1:PROBe 1", ":CHANnel2:PROBe 10",
        ":TRIGger:EDGE:SOURce CHANnel1", ":TRIGger:EDGE:SLOPe POSitive", ":TRIGger:EDGE:LEVel 1.65",
        ":TIMebase:MAIN:OFFSet 0", f":TIMebase:MAIN:SCALe {timebase_s}",
    ):
        scope.send(c)
    xinc = float(scope.query(":WAVeform:XINCrement?"))
    t0 = time.monotonic()
    with open(out, "w") as f:
        f.write("# elapsed_s offset_ns  (GPS->gen during lock acquisition; nan = out of window)\n")
        for _ in range(n):
            ok = scope.single()
            el = time.monotonic() - t0
            i1 = rising_edge(scope.waveform(1)) if ok else None
            i2 = rising_edge(scope.waveform(2)) if ok else None
            if i1 is None or i2 is None or not (15 < i2 < 985):
                f.write(f"{el:.1f} nan\n")
            else:
                f.write(f"{el:.1f} {(i2 - i1) * xinc * 1e9:.0f}\n")
            f.flush()
    print(f"wrote {out}")


def cmd_jitter(scope, out, timebase_s=20e-9, accumulate_s=6.0):
    """Show the gen-edge jitter as an infinite-persistence band at a fine timebase.

    Triggers on the GPS edge and centers a fine window on the gen edge (the GPS edge then sits
    off-screen left; the scope's `D` delay readout = the offset). Infinite persistence smears the
    gen rising edge over `accumulate_s` of free-run, so the *horizontal width of the gen-edge band
    IS the jitter*. Short accumulation keeps the slow sub-µs wander from broadening it much.
    """
    for c in (
        ":CHANnel1:DISPlay 1", ":CHANnel2:DISPlay 1",
        ":CHANnel1:PROBe 1", ":CHANnel2:PROBe 10",
        ":CHANnel1:SCALe 0.6", ":CHANnel2:SCALe 0.6",
        ":CHANnel1:OFFSet -2.1", ":CHANnel2:OFFSet -2.1",
        ":TRIGger:EDGE:SOURce CHANnel1", ":TRIGger:EDGE:SLOPe POSitive", ":TRIGger:EDGE:LEVel 1.65",
        ":DISPlay:GRADing:TIME MIN", ":TIMebase:MAIN:SCALe 1e-6", ":TIMebase:MAIN:OFFSet 0",
    ):
        scope.send(c)
    # find the current mean gen-vs-GPS offset, to center the fine window on the gen edge.
    xinc = float(scope.query(":WAVeform:XINCrement?"))
    offs = []
    for _ in range(10):
        if scope.single():
            i1 = rising_edge(scope.waveform(1))
            i2 = rising_edge(scope.waveform(2))
            if i1 is not None and i2 is not None and 15 < i2 < 985:
                offs.append((i2 - i1) * xinc)
    center = sum(offs) / len(offs) if offs else 0.0
    scope.send(f":TIMebase:MAIN:SCALe {timebase_s}")
    scope.send(f":TIMebase:MAIN:OFFSet {center}")  # gen edge at screen center; GPS off-screen left
    scope.send(":DISPlay:GRADing:TIME INFinite")
    scope.send(":CLEar")
    scope.send(":RUN")
    time.sleep(accumulate_s)
    scope.send(":STOP")
    n = scope.screenshot(out)
    scope.send(":DISPlay:GRADing:TIME MIN")  # restore non-persistent display
    print(f"center(offset)={center * 1e9:.0f}ns  wrote {out} ({n} bytes)")


def main():
    argv = sys.argv[1:]
    sub = argv[0] if argv else ""
    with Rigol() as scope:
        if sub == "phase":
            cmd_phase(
                scope,
                int(argv[1]) if len(argv) > 1 else 25,
                argv[2] if len(argv) > 2 else None,
            )
        elif sub == "capture":
            cmd_capture(
                scope,
                argv[1] if len(argv) > 1 else "scope-pps.png",
                float(argv[2]) if len(argv) > 2 else None,
            )
        elif sub == "converge":
            cmd_converge(scope, argv[1] if len(argv) > 1 else "converge.log")
        elif sub == "jitter":
            cmd_jitter(scope, argv[1] if len(argv) > 1 else "scope-jitter.png")
        else:
            sys.exit(
                "usage: scope_pps.py "
                "{phase [N] [log] | capture [out.png] [s/div] | converge [log] | jitter [out.png]}"
            )


if __name__ == "__main__":
    main()
