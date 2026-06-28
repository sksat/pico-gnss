#!/usr/bin/env python3
"""Combined per-PPS GIF: scope waveform (top) + firmware parameter time-series (bottom).

Each frame = one PPS: the live scope screenshot (output ch2 vs GPS ch1) on top, and a
growing multi-panel plot of firmware/scope params up to that PPS on the bottom, with the
current sample marked. The scope is captured every PPS via NORMal RUN + ch2-hash dedup
(see scope_raw); firmware params are read by tailing the live RTT log and snapshotting the
latest PPSGEN/TIME/SYNC values at each grab (aligned by wall-clock, good to sub-second).

Axes are FIXED to the full run's range (no jitter) via a two-pass flow: capture all PPS,
then render every frame with stable axes. The scope top AUTO-scales per frame (gradual
1-2-5 ladder, widen only when off-screen), so a from-boot run (sdiv_start=5000) shows the
pull-in. Reference/nominal dotted lines per panel (offset/err/ff_delta -> 0; ppb -> 0 =
nominal crystal). Capture is saved to a sidecar (<out>.frames/ + <out>.json) so the GIF can
be re-rendered without re-capturing (tune panels, then `render`).

Set RIGOL_HOST=<scope-ip>. Usage:
  scope_combo.py <rtt_log> <out.gif> [dur_s] [sdiv_start_ns]   # capture from a live cargo run
  scope_combo.py render <out.gif>                              # re-render from the saved sidecar
"""
import os, sys, io, re, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CLIP_NS = 100_000      # pre-lock hwphase/err garbage (e.g. -222e6) -> drop
CLIP_FF = 200_000      # ff_delta single lock-acquisition spikes (~-1.3e6) -> drop

# Each panel = (LEFT, RIGHT); RIGHT (twin y-axis) may be None.
# spec = (ylabel, [(legend, key, color)], ref, symlog):
#   ref    = nominal/zero dotted line (None = none)
#   symlog = linthresh (ns) for a symmetric-log y-axis (linear within +/-linthresh, log beyond),
#            so a 4-decade span like the offset pull-in (47us -> +/-tens ns) shows both ends; None = linear.
# ff_delta (temp-FF contribution) shares the temp panel on a twin axis since it is derived from temp.
PANELS = [
    (("offset / hwphase [ns]", [("scope offset", "off", "tab:blue"), ("fw hwphase", "hwphase", "tab:orange")], 0.0, 100.0), None),
    (("time err [ns]", [("err_ns", "err", "tab:purple")], 0.0, None), None),
    (("crystal ppb", [("ppb", "ppb", "tab:green")], 0.0, None), None),
    (("temp_raw (x256)", [("temp_raw", "temp", "tab:red")], None, None),
     ("ff_delta [mppb]", [("ff_delta", "ff_delta", "tab:brown")], 0.0, None)),
]


def kv(line, key):
    m = re.search(rf"{key}=(-?\d+)", line)
    return int(m.group(1)) if m else None


class LogTail:
    def __init__(self, path):
        self.f = open(path, "r", errors="replace")
        self.f.seek(0, 2)
        self.s = {}
    def poll(self):
        for line in self.f:
            if "PPSGEN count=" in line:
                for k in ("hwphase_ns", "temp_raw", "ff_delta", "trim_mppb", "slope_mppb"):
                    v = kv(line, k)
                    if v is not None:
                        self.s[k] = v
            elif "] TIME " in line:
                for k in ("ppb", "holdover_ms", "locked"):
                    v = kv(line, k)
                    if v is not None:
                        self.s[k] = v
            elif "] SYNC " in line:                 # err_ns / fire_ns / drift_us live here
                for k in ("err_ns", "fire_ns", "drift_us"):
                    v = kv(line, k)
                    if v is not None:
                        self.s[k] = v
        return dict(self.s)


def fixed_ylim(series, keys, ref):
    vals = [s[k] for s in series for k in keys if s.get(k) is not None]
    if ref is not None:
        vals.append(ref)
    if not vals:
        return (-1, 1)
    lo, hi = min(vals), max(vals)
    if hi == lo:
        return (lo - 1, hi + 1)
    pad = (hi - lo) * 0.08
    return (lo - pad, hi + pad)


def _draw_spec(ax, spec, sub, ts, yl):
    """Plot one axis's traces; returns the Line2D handles for the legend."""
    ylabel, traces, ref, symlog = spec
    if symlog:
        ax.set_yscale("symlog", linthresh=symlog)
    if ref is not None:
        ax.axhline(ref, ls=":", color="gray", lw=1.0, zorder=0)       # nominal / zero
    lines = []
    for name, key, color in traces:
        xy = [(t, s.get(key)) for t, s in zip(ts, sub) if s.get(key) is not None]
        if xy:
            xs, yy = zip(*xy)
            (ln,) = ax.plot(xs, yy, "-", color=color, lw=1.3, label=name)
            ax.plot(xs[-1], yy[-1], "o", color=color, ms=5)
            lines.append(ln)
    ax.set_ylabel(ylabel, fontsize=8)
    if yl:
        ax.set_ylim(*yl)
    ax.tick_params(labelsize=7)
    return lines


def render_panel(series, up_to, xlim, ylims):
    fig, axes = plt.subplots(len(PANELS), 1, figsize=(10.24, 1.7 * len(PANELS)), dpi=100, sharex=True)
    sub = series[:up_to + 1]
    ts = [s["t"] for s in sub]
    for ax, (left, right), (yl_l, yl_r) in zip(axes, PANELS, ylims):
        ax.set_xlim(*xlim); ax.grid(alpha=0.3)
        lines = _draw_spec(ax, left, sub, ts, yl_l)
        if right is not None:
            ax2 = ax.twinx()
            lines = lines + _draw_spec(ax2, right, sub, ts, yl_r)
        ax.legend(lines, [l.get_label() for l in lines], fontsize=7, loc="upper right", ncol=len(lines))
    axes[-1].set_xlabel("t since capture start [s]", fontsize=8)
    fig.tight_layout(pad=0.4)
    buf = io.BytesIO(); fig.savefig(buf, format="png"); plt.close(fig); buf.seek(0)
    return Image.open(buf).convert("RGB")


def _spec_ylim(series, spec):
    if spec is None:
        return None
    _, traces, ref, _ = spec
    return fixed_ylim(series, [k for _, k, _ in traces], ref)


def render_gif(series, png_bytes, out):
    W = 1024
    xlim = (0, max(s["t"] for s in series) or 1)
    ylims = [(_spec_ylim(series, left), _spec_ylim(series, right)) for left, right in PANELS]
    print(f"rendering {len(series)} frames; x={xlim} y={ylims}", flush=True)
    frames = []
    for i in range(len(series)):
        panel = render_panel(series, i, xlim, ylims)
        top = Image.open(io.BytesIO(png_bytes[i])).convert("RGB")
        if top.width != W:
            top = top.resize((W, round(top.height * W / top.width)))
        if panel.width != W:
            panel = panel.resize((W, round(panel.height * W / panel.width)))
        combo = Image.new("RGB", (W, top.height + panel.height), "white")
        combo.paste(top, (0, 0)); combo.paste(panel, (0, top.height))
        frames.append(combo.convert("P", palette=Image.ADAPTIVE))
    frames[0].save(out, save_all=True, append_images=frames[1:], duration=350, loop=0, optimize=True)
    print(f"GIF saved: {out} ({len(frames)} PPS frames)")


def save_sidecar(out, series, png_bytes):
    fdir = out + ".frames"
    os.makedirs(fdir, exist_ok=True)
    for i, b in enumerate(png_bytes):
        with open(os.path.join(fdir, f"{i:04d}.png"), "wb") as f:
            f.write(b)
    with open(out + ".json", "w") as f:
        json.dump(series, f)


def render_only(out):
    with open(out + ".json") as f:
        series = json.load(f)
    fdir = out + ".frames"
    png_bytes = [open(os.path.join(fdir, f"{i:04d}.png"), "rb").read() for i in range(len(series))]
    render_gif(series, png_bytes, out)


def capture(log, out, dur, sdiv0):
    from scope_raw import RawScope, rising_edge, setup, _step_toward, _pick_sdiv
    sc = RawScope(); setup(sc, sdiv0, ch3=False)
    sc.set_(":TRIGger:SWEep NORMal"); sc.set_(":RUN"); time.sleep(1.2)
    tail = LogTail(log)
    series = []; pngs = []; last_key = None; sdiv = sdiv0; applied = None
    t0 = time.time()
    while time.time() - t0 < dur and len(series) < 400:
        try:
            if applied != sdiv:
                sc.set_(f":TIMebase:MAIN:SCALe {sdiv:.3e}"); sc.set_(":TIMebase:MAIN:OFFSet 0"); applied = sdiv
                sc.drain(0.1); time.sleep(1.1); last_key = None
            b2 = sc.waveform(2); key = hash(bytes(b2))
            if key == last_key:
                tail.poll(); continue
            last_key = key
            e2 = rising_edge(b2)
            if e2 is None:
                if sdiv < 1e-5:
                    sdiv = _step_toward(sdiv, 1e-5)
                continue
            e1 = rising_edge(sc.waveform(1)); png = sc.screenshot_png()
            st = tail.poll()
            off = (e2 - e1) * (sdiv * 1e7) if e1 is not None else None
            hw = st.get("hwphase_ns"); hw = None if (hw is not None and abs(hw) > CLIP_NS) else hw
            er = st.get("err_ns"); er = None if (er is not None and abs(er) > CLIP_NS) else er
            ff = st.get("ff_delta"); ff = None if (ff is not None and abs(ff) > CLIP_FF) else ff
            series.append({"t": time.time() - t0, "off": off, "hwphase": hw, "err": er,
                           "ppb": st.get("ppb"), "ff_delta": ff, "temp": st.get("temp_raw"), "sdiv": sdiv})
            pngs.append(png)
            print(f"capture PPS#{len(series)} t={series[-1]['t']:.0f}s off={off} hw={hw} err={er} "
                  f"ppb={st.get('ppb')} ff={ff} {sdiv*1e9:.0f}ns/div", flush=True)
            if off is not None:
                sdiv = _step_toward(sdiv, _pick_sdiv(off))
        except Exception as ex:
            print(f"ERR {ex!r}", flush=True); sc.drain(0.3)
    sc.close()
    if not series:
        print("no frames"); return
    save_sidecar(out, series, pngs)        # so the GIF can be re-rendered without re-capturing
    render_gif(series, pngs, out)


if __name__ == "__main__":
    if sys.argv[1] == "render":
        render_only(sys.argv[2])
    else:
        log, out = sys.argv[1], sys.argv[2]
        dur = float(sys.argv[3]) if len(sys.argv) > 3 else 180.0
        sdiv0 = (float(sys.argv[4]) if len(sys.argv) > 4 else 50.0) * 1e-9
        capture(log, out, dur, sdiv0)
