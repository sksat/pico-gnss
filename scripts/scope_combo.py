#!/usr/bin/env python3
"""Combined per-PPS GIF: scope waveform (top) + firmware parameter time-series (bottom).

Each frame = one PPS: the live scope screenshot (output ch2 vs GPS ch1) on top, and a
growing multi-panel plot of {scope offset, firmware hwphase, ppb, temp_raw} up to that PPS
on the bottom, with the current sample marked. The scope is captured every PPS via NORMal
RUN + ch2-hash dedup (see scope_raw); the firmware params are read by tailing the live RTT
log (`cargo run` defmt output) and snapshotting the latest PPSGEN/TIME values at each grab.
The two streams are aligned by wall-clock (both live), good to sub-second for visualization.

Set RIGOL_HOST=<scope-ip>. Usage:
  scope_combo.py <rtt_log> <out.gif> [dur_s] [sdiv_ns]
Run it alongside a live `cargo run > <rtt_log>` so the log is being written.
"""
import os, sys, io, re, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scope_raw import RawScope, rising_edge, setup
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOG = sys.argv[1]
OUT = sys.argv[2]
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 180.0
SDIV = (float(sys.argv[4]) if len(sys.argv) > 4 else 50.0) * 1e-9


def kv(line, key):
    m = re.search(rf"{key}=(-?\d+)", line)
    return int(m.group(1)) if m else None


class LogTail:
    """Tail a live RTT log; keep the latest per-PPS params."""
    def __init__(self, path):
        self.f = open(path, "r", errors="replace")
        self.f.seek(0, 2)               # start at end -> only new lines
        self.s = {}

    def poll(self):
        for line in self.f:
            if "PPSGEN count=" in line:
                for k in ("hwphase_ns", "temp_raw", "trim_mppb", "slope_mppb", "dev_ns"):
                    v = kv(line, k)
                    if v is not None:
                        self.s[k] = v
            elif "] TIME " in line:
                for k in ("ppb", "err_ns", "holdover_ms", "locked"):
                    v = kv(line, k)
                    if v is not None:
                        self.s[k] = v
        return dict(self.s)


# expanding (never-shrinking) y-limits so the animated axes don't jitter
class YLim:
    def __init__(self):
        self.lo = None; self.hi = None
    def add(self, *vals):
        for v in vals:
            if v is None:
                continue
            self.lo = v if self.lo is None else min(self.lo, v)
            self.hi = v if self.hi is None else max(self.hi, v)
    def lim(self):
        if self.lo is None:
            return (-1, 1)
        if self.hi == self.lo:
            return (self.lo - 1, self.hi + 1)
        pad = (self.hi - self.lo) * 0.1
        return (self.lo - pad, self.hi + pad)


PANELS = [
    ("offset / hwphase [ns]", [("scope offset", "off", "tab:blue"), ("fw hwphase", "hwphase", "tab:orange")]),
    ("crystal ppb", [("ppb", "ppb", "tab:green")]),
    ("temp_raw (x256)", [("temp", "temp", "tab:red")]),
]


def render_panel(series, dur, ylims):
    fig, axes = plt.subplots(len(PANELS), 1, figsize=(10.24, 4.4), dpi=100, sharex=True)
    ts = [s["t"] for s in series]
    for ax, (label, traces), yl in zip(axes, PANELS, ylims):
        for name, key, color in traces:
            ys = [s.get(key) for s in series]
            xy = [(t, y) for t, y in zip(ts, ys) if y is not None]
            if xy:
                xs, yy = zip(*xy)
                ax.plot(xs, yy, "-", color=color, lw=1.3, label=name)
                ax.plot(xs[-1], yy[-1], "o", color=color, ms=5)      # current sample
        ax.set_ylabel(label, fontsize=8)
        ax.set_xlim(0, dur)
        ax.set_ylim(*yl.lim())
        ax.grid(alpha=0.3)
        ax.tick_params(labelsize=7)
        ax.legend(fontsize=7, loc="upper right", ncol=len(traces))
    axes[-1].set_xlabel("t since capture start [s]", fontsize=8)
    fig.tight_layout(pad=0.4)
    buf = io.BytesIO()
    fig.savefig(buf, format="png")
    plt.close(fig)
    buf.seek(0)
    return Image.open(buf).convert("RGB")


def main():
    sc = RawScope(); setup(sc, SDIV, ch3=False)
    sc.set_(":TRIGger:SWEep NORMal"); sc.set_(":RUN"); time.sleep(1.2)
    tail = LogTail(LOG)
    xinc = SDIV * 1e7
    series = []; frames = []; last_key = None
    ylims = [YLim() for _ in PANELS]
    W = 1024
    t0 = time.time()
    while time.time() - t0 < DUR and len(frames) < 400:
        try:
            b2 = sc.waveform(2); key = hash(bytes(b2))
            if key == last_key:
                tail.poll(); continue
            last_key = key
            e2 = rising_edge(b2)
            if e2 is None:
                continue
            e1 = rising_edge(sc.waveform(1)); png = sc.screenshot_png()
            st = tail.poll()
            off = (e2 - e1) * xinc if e1 is not None else None
            rec = {"t": time.time() - t0, "off": off,
                   "hwphase": st.get("hwphase_ns"), "ppb": st.get("ppb"),
                   "temp": st.get("temp_raw"), "err": st.get("err_ns")}
            series.append(rec)
            ylims[0].add(off, rec["hwphase"]); ylims[1].add(rec["ppb"]); ylims[2].add(rec["temp"])
            panel = render_panel(series, DUR, ylims)
            top = Image.open(io.BytesIO(png)).convert("RGB")
            if top.width != W:
                top = top.resize((W, round(top.height * W / top.width)))
            if panel.width != W:
                panel = panel.resize((W, round(panel.height * W / panel.width)))
            combo = Image.new("RGB", (W, top.height + panel.height), "white")
            combo.paste(top, (0, 0)); combo.paste(panel, (0, top.height))
            frames.append(combo.convert("P", palette=Image.ADAPTIVE))
            print(f"PPS#{len(frames)} t={rec['t']:.0f}s off={off} hwphase={rec['hwphase']} "
                  f"ppb={rec['ppb']} temp={rec['temp']}", flush=True)
        except Exception as ex:
            print(f"ERR {ex!r}", flush=True); sc.drain(0.3)
    sc.close()
    if frames:
        frames[0].save(OUT, save_all=True, append_images=frames[1:], duration=350, loop=0, optimize=True)
        print(f"GIF saved: {OUT} ({len(frames)} PPS frames)")
    else:
        print("no frames")


if __name__ == "__main__":
    main()
