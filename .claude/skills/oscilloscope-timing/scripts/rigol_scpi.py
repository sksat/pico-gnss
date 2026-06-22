#!/usr/bin/env python3
"""Rigol DHO800 SCPI helper for timing / phase measurement between two channels.

Zero dependencies (stdlib socket only). The scope host is read from the RIGOL_HOST
environment variable — never hardcode an IP in a committed file.

  RIGOL_HOST=<scope-ip> python3 rigol_scpi.py phase <ref_ch> <sig_ch> [N] [log]
  RIGOL_HOST=<scope-ip> python3 rigol_scpi.py capture <out.png> [s/div]

`phase` triggers on the reference channel's rising edge, dumps both channels' waveforms,
and reports the sig-vs-ref rising-edge offset over N single-shots as mean / sigma / pp.
Reporting the *distribution* (not one shot) is the whole point: a timing offset that
drifts (temperature, a receiver's PPS sawtooth, a wandering source) makes any single
snapshot a random draw, not "the offset". See SKILL.md.

Why dump the waveform instead of the scope's built-in delay/period measurement: a wide
pulse (e.g. a ~100 ms 1PPS) in a µs window has only one visible edge, so the built-in
edge-pairing fails. Dumping the trace and finding the crossing ourselves is robust.

Other models: the socket/SCPI shape is generic; the command strings here are DHO800
(12-bit). For DS1000Z etc. adjust :WAVeform:MODE/FORMat and the block-read, and check
:MEASure:CLEar argument handling. See SKILL.md "Porting to another model".
"""
import os
import socket
import statistics
import sys
import time

PORT = int(os.environ.get("RIGOL_PORT", "5555"))


class Rigol:
    """Minimal SCPI client for a Rigol DHO800-series scope over raw TCP."""

    def __init__(self, host=None, port=PORT, timeout=4.0):
        host = host or os.environ.get("RIGOL_HOST")
        if not host:
            raise SystemExit("set RIGOL_HOST=<scope-ip> (do not hardcode the IP in a committed file)")
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
        """Read the SCPI error queue empty. It is FIFO — drain fully before blaming a command."""
        out = []
        for _ in range(32):
            e = self.query(":SYSTem:ERRor?")
            if e.startswith("0,") or "No error" in e:
                break
            out.append(e)
        return out

    def waveform(self, ch, mode="NORMal"):
        """Return the channel's screen samples as BYTE values (≈ 0..255)."""
        self.send(f":WAVeform:SOURce CHANnel{ch}")
        self.send(f":WAVeform:MODE {mode}")
        self.send(":WAVeform:FORMat BYTE")
        return self.query_block(":WAVeform:DATA?")

    def single(self, settle=0.05, tries=80):
        """Arm a single acquisition and poll until the scope stops (triggered). True if it stopped."""
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
        return len(path) and len(png)


def rising_edge(samples, min_swing=100):
    """Sub-sample index of the first rising mid-level crossing, or None.

    `samples` are BYTE values (screen ≈ 0..255). `min_swing` rejects baseline ringing: a real
    0->full-scale edge swings ~170 levels, while ringing when the real edge is off-screen is only
    ~30, so a high min_swing avoids latching noise as a false edge (which would yield a bogus gap).

    This is a deliberately simple helper for clean on-screen edges: it takes the 50% point between
    the window's min and max. Overshoot / ringing / multiple edges shift that midpoint, and a
    badly-scaled (small on screen) but valid signal is rejected by min_swing. For rigorous edge
    timing use a fixed threshold with hysteresis, a max-slope estimate, or known logic levels.
    """
    lo, hi = min(samples), max(samples)
    if hi - lo < min_swing:
        return None
    thr = (lo + hi) / 2
    for i in range(1, len(samples)):
        if samples[i - 1] < thr <= samples[i]:
            return (i - 1) + (thr - samples[i - 1]) / (samples[i] - samples[i - 1])
    return None


def measure_edge_offset(scope, ref_ch, sig_ch, n=25, level=1.65, edge_margin=15,
                        timebase_s=None, logpath=None):
    """Trigger on `ref_ch`'s rising edge, dump both channels, report sig-vs-ref offset over N shots.

    Returns the list of per-shot offsets in seconds (sig − ref; negative = sig leads ref). Prints
    mean / sigma / pp. The distribution is the result, not any single value — see the module docstring.

    timebase_s (s/div): if given, set it before measuring. The time resolution comes from s/div via
    XINCrement, so a coarse window inflates sigma by quantization — set it so both edges fit, then
    tighten. If None, the scope's current timebase is used as-is (this function does NOT change it).

    edge_margin drops shots where EITHER edge landed within `edge_margin` samples of the screen edge
    (its crossing index is then unreliable, e.g. with a time offset), rather than logging a spike.
    """
    for c in (
        f":TRIGger:EDGE:SOURce CHANnel{ref_ch}",
        ":TRIGger:EDGE:SLOPe POSitive",
        f":TRIGger:EDGE:LEVel {level}",
        ":TRIGger:SWEep NORMal",
    ):
        scope.send(c)
    if timebase_s is not None:
        scope.send(f":TIMebase:MAIN:SCALe {timebase_s}")
    # Read XINCrement for the waveform we will actually dump: set MODE/FORMat first so the value
    # reflects this screen-waveform, not a stale mode's sample spacing.
    scope.send(":WAVeform:MODE NORMal")
    scope.send(":WAVeform:FORMat BYTE")
    xinc = float(scope.query(":WAVeform:XINCrement?"))  # seconds per sample at the current timebase

    deltas = []
    for _ in range(n):
        if not scope.single():
            continue
        wr = scope.waveform(ref_ch)
        ws = scope.waveform(sig_ch)
        ir = rising_edge(wr)
        isg = rising_edge(ws)
        if ir is None or isg is None:
            continue
        # Both crossings must be clear of the screen edges, not just the signal one.
        if not (edge_margin < ir < len(wr) - edge_margin):
            continue
        if not (edge_margin < isg < len(ws) - edge_margin):
            continue
        deltas.append((isg - ir) * xinc)

    print(f"xinc={xinc * 1e9:.3f} ns/pt  N_ok={len(deltas)}/{n}")
    if len(deltas) > 1:
        m, sd = statistics.mean(deltas), statistics.pstdev(deltas)
        print(f"offset(ch{sig_ch}-ch{ref_ch}): mean={m * 1e9:.1f}ns  sigma={sd * 1e9:.1f}ns  "
              f"min={min(deltas) * 1e9:.0f}  max={max(deltas) * 1e9:.0f}  pp={(max(deltas) - min(deltas)) * 1e9:.0f}ns")
    elif deltas:
        print(f"offset(ch{sig_ch}-ch{ref_ch}): {deltas[0] * 1e9:.1f}ns (only 1 shot — NOT a distribution)")
    else:
        print("no valid edge pairs (both edges on-screen? trigger level? probe attenuation?)")
    if logpath:
        with open(logpath, "w") as f:
            f.write(f"# offset ch{sig_ch}-ch{ref_ch} [ns], one rising-edge pair per single-shot\n")
            for d in deltas:
                f.write(f"{d * 1e9:.1f}\n")
        print(f"wrote {logpath} ({len(deltas)} samples)")
    return deltas


def _cmd_capture(scope, out, timebase_s=None):
    """Single triggered screenshot for a quick visual record. timebase_s overrides s/div."""
    if timebase_s:
        scope.send(f":TIMebase:MAIN:SCALe {timebase_s}")
    errs = scope.drain_errors()
    if errs:
        print("scope errors:", errs)
    if not scope.single():
        print("warning: no trigger (is the reference edge present and the level/probe right?)")
    print(f"wrote {out} ({scope.screenshot(out)} bytes)")


def main():
    argv = sys.argv[1:]
    sub = argv[0] if argv else ""
    with Rigol() as scope:
        if sub == "phase" and len(argv) >= 3:
            measure_edge_offset(
                scope, int(argv[1]), int(argv[2]),
                n=int(argv[3]) if len(argv) > 3 else 25,
                logpath=argv[4] if len(argv) > 4 else None,
            )
        elif sub == "capture":
            _cmd_capture(
                scope,
                argv[1] if len(argv) > 1 else "scope.png",
                float(argv[2]) if len(argv) > 2 else None,
            )
        else:
            sys.exit("usage: rigol_scpi.py {phase <ref_ch> <sig_ch> [N] [log] | capture [out.png] [s/div]}")


if __name__ == "__main__":
    main()
