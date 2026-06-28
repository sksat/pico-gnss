#!/usr/bin/env python3
"""Rigol DHO800 phase measurement / screenshots over the raw LXI socket (port 5555).

This is the *fallback* path for when the scope's VXI-11 read wedges (see
`scripts/rigol_vxi11.py` and the rigol recovery notes): VXI-11 `device clear` succeeds but
every read times out, and the scope is unusable over VXI-11 for the session. The raw socket
still works, but its SCPI is LAZY-FLUSH and hostile:

  - A command pushes all PREVIOUSLY-pending responses to the socket; the command's own
    response stays pending until the NEXT command (so a naive read lags by one, and a drain
    finds nothing because the pending response is not on the wire yet).
  - `*CLS` is a no-reply command that flushes pending TEXT, so it makes a clean pusher for
    instant text queries: `send(cmd); send("*CLS"); readline()` returns cmd's response and
    leaves the queue empty.
  - BUT `*CLS` ABORTS a pending waveform block or measurement. For a block, push it with a
    real query (`:WAVeform:FORMat?`) instead, then skip-to-'#'. Its text response is left
    pending and cleared by the next setup command's flush (+ a drain before the next block).
  - During acquisition do NOT use `*CLS` (it clears the captured frame; observed with
    :SINGle polling). Per shot use `:SINGle` + a fixed wait for one fresh triggered frame,
    and verify the trigger fired (ch1 edge present) rather than polling status.

Channels (this rig): ch1 = GPS-R PPS (trigger + reference), ch2 = GP4 output loopback (x10),
ch3 = GP2 GPS-at-Pico. ch1 ~= ch3 (path delay tiny), so for output-vs-GPS we use ch1 as the
reference and leave ch3 off — cleaner screenshots and a less noisy edge than ch3's small
signal. Edge timing is amplitude-independent. NORMal waveform mode returns ~1000 pts / 10
div, so xinc(ns/pt) = s_per_div * 1e7.

Set RIGOL_HOST=<scope-ip>. Usage:
  scope_raw.py phase [N] [sdiv_ns]      # output(ch2)-vs-GPS(ch1): mean/std/%<=100 over N shots
  scope_raw.py shot  <out.png> [sdiv_ns]# one triggered screenshot (ch1/ch2, ch3 off)
  scope_raw.py convergence <out.gif> [dur_s]  # post-boot pull-in GIF, AUTO timescale per frame
"""
import os, sys, socket, time, math, statistics as st

HOST = os.environ.get("RIGOL_HOST")
if not HOST:
    raise SystemExit("set RIGOL_HOST=<scope-ip>")


class RawScope:
    def __init__(self):
        self.s = socket.create_connection((HOST, 5555), timeout=8)
        for _ in range(12):                         # empty the lazy-flush backlog
            self._send("*CLS")
            if not self.drain(0.3):
                break

    def _send(self, c):
        self.s.sendall((c + "\n").encode())

    def drain(self, t):
        self.s.settimeout(t); g = b""
        try:
            while True:
                d = self.s.recv(65536)
                if not d:
                    break
                g += d
        except socket.timeout:
            pass
        return g

    def _readline(self, to=4):
        b = b""; self.s.settimeout(to)
        while True:
            c = self.s.recv(1)
            if not c:
                break
            if c == b"\n":
                if b.strip() == b"":
                    b = b""; continue
                break
            b += c
        return b.decode(errors="replace").strip()

    def _rexact(self, n, to=8):
        b = b""; self.s.settimeout(to)
        while len(b) < n:
            c = self.s.recv(n - len(b))
            if not c:
                raise EOFError
            b += c
        return b

    def qtext(self, cmd):
        """Instant text query: *CLS pusher flushes the (already-generated) response."""
        self._send(cmd); self._send("*CLS")
        return self._readline()

    def set_(self, cmd):
        self._send(cmd)

    def _block(self):
        for _ in range(4096):
            if self._rexact(1) == b"#":
                break
        else:
            raise RuntimeError("no block header")
        nd = int(self._rexact(1)); ln = int(self._rexact(nd))
        return self._rexact(ln)

    def waveform(self, ch):
        self._send(f":WAVeform:SOURce CHANnel{ch}")
        self._send(":WAVeform:MODE NORMal")
        self._send(":WAVeform:FORMat BYTE")
        self.drain(0.15)                            # clear stale text the settings flushed
        self._send(":WAVeform:DATA?")
        self._send(":WAVeform:FORMat?")             # real-query pusher flushes the block
        return list(self._block())

    def screenshot_png(self):
        self._send(":DISPlay:DATA? PNG")
        self._send(":WAVeform:FORMat?")             # real-query pusher flushes the PNG block
        png = bytes(self._block()); self.drain(0.2)
        return png

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


def rising_edge(w):
    lo, hi = min(w), max(w)
    if hi - lo < 30:
        return None
    th = (lo + hi) / 2
    for i in range(1, len(w)):
        if w[i - 1] < th <= w[i]:
            return (i - 1) + (th - w[i - 1]) / (w[i] - w[i - 1])
    return None


def setup(sc, sdiv, ch3=False):
    sc.set_(":CHANnel1:DISPlay ON"); sc.set_(":CHANnel1:SCALe 1"); sc.set_(":CHANnel1:OFFSet -1.5")
    sc.set_(":CHANnel2:DISPlay ON"); sc.set_(":CHANnel2:SCALe 1"); sc.set_(":CHANnel2:OFFSet -1.5")
    sc.set_(":CHANnel3:DISPlay " + ("ON" if ch3 else "OFF"))
    sc.set_(":TRIGger:EDGE:SOURce CHANnel1"); sc.set_(":TRIGger:EDGE:LEVel 1.65"); sc.set_(":TRIGger:EDGE:SLOPe POSitive")
    sc.set_(f":TIMebase:MAIN:SCALe {sdiv:.3e}"); sc.set_(":TIMebase:MAIN:OFFSet 0")
    sc.set_(":TRIGger:SWEep SINGle")
    time.sleep(0.3)


def triggered(sc):
    """One fresh triggered frame; ch1 (trigger source) edge present == trigger fired."""
    for _ in range(5):
        sc.set_(":SINGle"); time.sleep(1.4)
        b1 = sc.waveform(1)
        if rising_edge(b1) is not None:
            return b1, sc.waveform(2)
        sc.drain(0.2)
    return b1, None


def cmd_phase(argv):
    n = int(argv[0]) if argv else 20
    sdiv = (float(argv[1]) if len(argv) > 1 else 50.0) * 1e-9
    sc = RawScope(); setup(sc, sdiv)
    xinc = sdiv * 1e7
    off = []; clip = 0
    for _ in range(n):
        b1, b2 = triggered(sc)
        e1, e2 = rising_edge(b1), (rising_edge(b2) if b2 else None)
        if None not in (e1, e2):
            off.append((e2 - e1) * xinc)
        else:
            clip += 1
    sc.close()
    if off:
        print(f"OUT-vs-GPS mean={st.mean(off):+.1f} std={st.pstdev(off):.1f} n={len(off)} clip={clip} "
              f"min={min(off):+.0f} max={max(off):+.0f} |<=100|={100*sum(1 for x in off if abs(x)<=100)/len(off):.0f}% "
              f"xinc={xinc:.2f}ns")
    else:
        print(f"MEASURE FAIL clip={clip}")


def cmd_shot(argv):
    out = argv[0]
    sdiv = (float(argv[1]) if len(argv) > 1 else 50.0) * 1e-9
    sc = RawScope(); setup(sc, sdiv)
    b1, _ = triggered(sc)
    png = sc.screenshot_png()
    sc.close()
    with open(out, "wb") as f:
        f.write(png)
    print(f"saved {out} ({len(png)} bytes, {sdiv*1e9:.0f} ns/div)")


# 1-2-5 timebase ladder (ns/div), 10 ns/div .. 10 us/div.
LADDER = [10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000]

def _pick_sdiv(off_ns):
    """Ideal ns/div so |offset| ~= 3 divisions, snapped up the 1-2-5 ladder."""
    want = max(abs(off_ns) / 3.0, LADDER[0])
    for v in LADDER:
        if want <= v:
            return v * 1e-9
    return LADDER[-1] * 1e-9

def _step_toward(cur_s, tgt_s):
    """Move the timebase by at most ONE ladder notch toward the target, so the GIF zooms
    gradually instead of jumping decades per frame."""
    def idx(s):
        ns = s * 1e9
        return min(range(len(LADDER)), key=lambda i: abs(LADDER[i] - ns))
    ci, ti = idx(cur_s), idx(tgt_s)
    if ti > ci:
        ci += 1
    elif ti < ci:
        ci -= 1
    return LADDER[ci] * 1e-9


def cmd_convergence(argv):
    from PIL import Image, ImageDraw, ImageFont
    out = argv[0]
    dur = float(argv[1]) if len(argv) > 1 else 280.0
    interval = 4.5                                    # denser frames so the 1-notch zoom can follow
    sc = RawScope(); setup(sc, 5e-6)                 # start wide (5 us/div)
    try:
        font = ImageFont.truetype("/usr/share/fonts/TTF/DejaVuSans-Bold.ttf", 22)
    except Exception:
        font = ImageFont.load_default()

    def capture(sdiv):
        """One on-screen triggered frame at sdiv. Only WIDEN (fast) if the edge is off the
        screen, so we never lose it; never zoom in here -- the gradual zoom is between frames."""
        for _ in range(8):
            sc.set_(f":TIMebase:MAIN:SCALe {sdiv:.3e}"); sc.set_(":TIMebase:MAIN:OFFSet 0")
            sc.set_(":SINGle"); time.sleep(1.4)
            png = sc.screenshot_png(); b1 = sc.waveform(1)
            if rising_edge(b1) is None:
                sc.drain(0.2); continue                 # trigger not fired -> retry
            e1 = rising_edge(b1); e2 = rising_edge(sc.waveform(2))
            if e2 is None and sdiv < 1e-5:
                sdiv = _step_toward(sdiv, 1e-5); continue   # off-screen -> widen one notch & retry
            off = (e2 - e1) * (sdiv * 1e7) if e2 is not None else None
            return png, off, sdiv
        return png, None, sdiv

    from io import BytesIO
    frames = []; sdiv = 5e-6; t0 = time.time()
    while time.time() - t0 < dur:
        el = time.time() - t0
        try:                                          # a transient scope timeout must not kill the run
            png, off, sdiv = capture(sdiv)
            img = Image.open(BytesIO(png)).convert("RGB"); d = ImageDraw.Draw(img)
            if off is not None:
                txt = f"t={el:5.0f}s   offset={off:+8.1f} ns   {sdiv*1e9:.0f} ns/div"
                sdiv = _step_toward(sdiv, _pick_sdiv(off))   # gradual: <=1 ladder notch / frame
            else:
                txt = f"t={el:5.0f}s   offset=off-screen   {sdiv*1e9:.0f} ns/div"
            d.rectangle([8, 60, 780, 96], fill=(0, 0, 0)); d.text((14, 62), txt, fill=(255, 255, 0), font=font)
            frames.append(img.convert("P", palette=Image.ADAPTIVE)); print(txt, flush=True)
        except Exception as ex:
            print(f"t={el:5.0f}s ERR {ex!r}", flush=True); sc.drain(0.4)
        time.sleep(interval)
    sc.close()
    if frames:
        frames[0].save(out, save_all=True, append_images=frames[1:], duration=350, loop=0, optimize=True)
        print(f"GIF saved: {out} ({len(frames)} frames)")


def cmd_wander(argv):
    """Every-PPS wander GIF. The serial :SINGle readout floor (~1.5-2 s) can't hit 1 frame/PPS,
    so instead run NORMal sweep (the scope auto-triggers on every PPS) and grab back-to-back
    while RUNning. A waveform grab is ~0.4 s < the 1 s PPS period, so every PPS is caught,
    usually 2-3x; dedup by the ch2 waveform bytes so exactly one frame per PPS enters the GIF."""
    from io import BytesIO
    from PIL import Image, ImageDraw, ImageFont
    out = argv[0]
    n = int(argv[1]) if len(argv) > 1 else 20
    sdiv = (float(argv[2]) if len(argv) > 2 else 50.0) * 1e-9
    sc = RawScope(); setup(sc, sdiv, ch3=False)
    sc.set_(":TRIGger:SWEep NORMal"); sc.set_(":RUN"); time.sleep(1.2)
    try:
        font = ImageFont.truetype("/usr/share/fonts/TTF/DejaVuSans-Bold.ttf", 22)
    except Exception:
        font = ImageFont.load_default()
    xinc = sdiv * 1e7
    frames = []; offs = []; last_key = None; t_end = time.time() + n * 1.5 + 30
    while len(frames) < n and time.time() < t_end:
        try:
            b2 = sc.waveform(2)                       # cheap read first, for dedup + edge
            key = hash(bytes(b2))
            if key == last_key:                       # same PPS frame already captured
                continue
            e2 = rising_edge(b2)
            if e2 is None:
                continue
            last_key = key
            e1 = rising_edge(sc.waveform(1))
            png = sc.screenshot_png()
            off = (e2 - e1) * xinc if e1 is not None else None
            img = Image.open(BytesIO(png)).convert("RGB"); d = ImageDraw.Draw(img)
            txt = (f"PPS #{len(frames)+1}   offset={off:+7.1f} ns   {sdiv*1e9:.0f} ns/div"
                   if off is not None else f"PPS #{len(frames)+1}   {sdiv*1e9:.0f} ns/div")
            d.rectangle([8, 60, 660, 96], fill=(0, 0, 0)); d.text((14, 62), txt, fill=(255, 255, 0), font=font)
            frames.append(img.convert("P", palette=Image.ADAPTIVE))
            if off is not None:
                offs.append(off)
            print(txt, flush=True)
        except Exception as ex:
            print(f"ERR {ex!r}", flush=True); sc.drain(0.3)
    sc.close()
    if frames:
        frames[0].save(out, save_all=True, append_images=frames[1:], duration=350, loop=0, optimize=True)
        if offs:
            print(f"GIF saved: {out} ({len(frames)} PPS frames) mean={st.mean(offs):+.1f} std={st.pstdev(offs):.1f} ns")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); raise SystemExit(2)
    {"phase": cmd_phase, "shot": cmd_shot, "convergence": cmd_convergence, "wander": cmd_wander}[sys.argv[1]](sys.argv[2:])
