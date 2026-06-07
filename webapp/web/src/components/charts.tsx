import { Canvas } from "./Canvas";
import { snrColor, sysColor, SYS_INFO } from "../nmea";
import type { Accuracy, Timing } from "../stats";
import type { GnssState } from "../types";

// ---------- 汎用描画ヘルパ ----------
function gridY(ctx: CanvasRenderingContext2D, w: number, h: number, vals: number[], fmt: (v: number) => string, max: number, min = 0) {
  ctx.strokeStyle = "#1a2530";
  ctx.fillStyle = "#475569";
  ctx.font = "9px ui-monospace, monospace";
  ctx.textAlign = "left";
  for (const v of vals) {
    const y = h * (1 - (v - min) / (max - min));
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
    ctx.fillText(fmt(v), 2, y - 2);
  }
}

function drawLine(
  ctx: CanvasRenderingContext2D, w: number, h: number,
  data: (number | null)[], color: string, yMin: number, yMax: number, pad = 4,
) {
  if (yMax - yMin < 1e-9) yMax = yMin + 1;
  const n = data.length;
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  let started = false;
  data.forEach((d, i) => {
    if (d == null) {
      started = false;
      return;
    }
    const x = n > 1 ? (i / (n - 1)) * w : w / 2;
    const y = pad + (h - 2 * pad) * (1 - (d - yMin) / (yMax - yMin));
    if (!started) {
      ctx.moveTo(x, y);
      started = true;
    } else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

function rangeOf(arrs: (number | null)[][], fallback: [number, number]): [number, number] {
  const xs = arrs.flat().filter((x): x is number => x != null);
  if (xs.length === 0) return fallback;
  let lo = Math.min(...xs);
  let hi = Math.max(...xs);
  if (hi - lo < 1e-6) { lo -= 1; hi += 1; }
  const m = (hi - lo) * 0.1;
  return [lo - m, hi + m];
}

// ---------- PPS sparkline ----------
export function drawSpark(ctx: CanvasRenderingContext2D, w: number, h: number, dev: number[]) {
  if (dev.length < 2) return;
  const data = dev.slice(-80);
  const maxAbs = Math.max(50, ...data.map((d) => Math.abs(d)));
  const mid = h / 2;
  ctx.strokeStyle = "#22303d";
  ctx.beginPath(); ctx.moveTo(0, mid); ctx.lineTo(w, mid); ctx.stroke();
  drawLine(ctx, w, h, data, "#36d399", -maxAbs, maxAbs, 3);
}

// ---------- Sky plot ----------
export function SkyPlot({ s }: { s: GnssState }) {
  const present = [...new Set(s.sats.map((x) => x.sys))];
  const tracked = s.sats.filter((x) => x.snr != null && x.snr > 0).length;
  return (
    <section className="panel a-sky">
      <h2>Sky plot <span className="hdr-aux">{s.sats.length} in view · {tracked} tracked</span></h2>
      <Canvas className="canvas-sky" draw={(ctx, w, h) => {
        const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 16;
        ctx.strokeStyle = "#1f2a36"; ctx.fillStyle = "#5a6b7b";
        ctx.font = "10px ui-monospace, monospace"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
        for (const e of [0, 30, 60]) { ctx.beginPath(); ctx.arc(cx, cy, (R * (90 - e)) / 90, 0, Math.PI * 2); ctx.stroke(); }
        ctx.beginPath(); ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R); ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy); ctx.stroke();
        for (const [l, dx, dy] of [["N", 0, -R - 8], ["S", 0, R + 8], ["E", R + 8, 0], ["W", -R - 8, 0]] as const)
          ctx.fillText(l, cx + dx, cy + dy);
        for (const sat of s.sats) {
          if (sat.elev == null || sat.azim == null) continue;
          const r = (R * (90 - Math.max(0, Math.min(90, sat.elev)))) / 90;
          const a = (sat.azim * Math.PI) / 180;
          const x = cx + r * Math.sin(a), y = cy - r * Math.cos(a);
          const trk = sat.snr != null && sat.snr > 0;
          ctx.beginPath(); ctx.arc(x, y, trk ? 8 : 4, 0, Math.PI * 2);
          ctx.fillStyle = trk ? snrColor(sat.snr) : "#33414f"; ctx.fill();
          if (s.usedPrn.has(sat.prn)) { ctx.lineWidth = 2; ctx.strokeStyle = sysColor(sat.sys); ctx.stroke(); }
          if (trk) { ctx.fillStyle = "#0b0f14"; ctx.font = "9px ui-monospace, monospace"; ctx.fillText(String(sat.prn), x, y); }
        }
      }} />
      <div className="legend">
        {present.map((sys) => (
          <span key={sys}><i style={{ background: sysColor(sys) }} />{SYS_INFO[sys]?.name ?? sys}</span>
        ))}
      </div>
    </section>
  );
}

// ---------- SNR bars ----------
export function SnrChart({ s }: { s: GnssState }) {
  const list = s.sats.filter((x) => x.snr != null && x.snr > 0)
    .sort((a, b) => (a.sys === b.sys ? a.prn - b.prn : a.sys.localeCompare(b.sys)));
  return (
    <section className="panel a-snr">
      <h2>C/N₀ (SNR) <span className="hdr-aux">{list.length} tracked</span></h2>
      <Canvas className="canvas-snr" draw={(ctx, w, h) => {
        const padB = 26, padT = 8, maxSnr = 55;
        gridY(ctx, w, h - padB - padT, [20, 30, 40, 50].map((g) => g), (v) => String(v), maxSnr);
        const n = list.length || 1, bw = Math.min(34, (w - 8) / n);
        list.forEach((sat, i) => {
          const snr = sat.snr!;
          const bh = (h - padT - padB) * Math.min(1, snr / maxSnr);
          const x = 4 + i * bw, y = h - padB - bh;
          ctx.fillStyle = snrColor(snr); ctx.fillRect(x + 1, y, bw - 2, bh);
          if (s.usedPrn.has(sat.prn)) { ctx.fillStyle = sysColor(sat.sys); ctx.fillRect(x + 1, y - 3, bw - 2, 2); }
          ctx.save(); ctx.translate(x + bw / 2, h - padB + 3); ctx.rotate(-Math.PI / 2);
          ctx.fillStyle = "#7a8a9a"; ctx.font = "9px ui-monospace, monospace";
          ctx.textAlign = "right"; ctx.textBaseline = "middle";
          ctx.fillText(`${SYS_INFO[sat.sys]?.name.slice(0, 2) ?? sat.sys}${sat.prn}`, 0, 0); ctx.restore();
        });
      }} />
    </section>
  );
}

// ---------- Position accuracy (scatter) ----------
export function AccuracyPanel({ s, acc }: { s: GnssState; acc: Accuracy }) {
  const f = s.fix;
  return (
    <section className="panel">
      <h2>Position accuracy <span className="hdr-aux">{acc.n}/{acc.total} samples · ~2 min window</span></h2>
      <Canvas className="canvas-scatter" draw={(ctx, w, h) => {
        const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 10;
        const maxR = Math.max(acc.r95 * 1.25, 2, ...acc.pts.map((p) => Math.hypot(p.e, p.n)));
        const sc = R / maxR;
        ctx.strokeStyle = "#1f2a36"; ctx.fillStyle = "#5a6b7b";
        ctx.font = "9px ui-monospace, monospace"; ctx.textAlign = "left"; ctx.textBaseline = "top";
        ctx.beginPath(); ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R); ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy); ctx.stroke();
        for (const [rr, col, lab] of [[acc.cep, "#36d399", "CEP"], [acc.r95, "#f59e0b", "R95"]] as const) {
          if (rr <= 0) continue;
          ctx.strokeStyle = col; ctx.beginPath(); ctx.arc(cx, cy, rr * sc, 0, Math.PI * 2); ctx.stroke();
          ctx.fillStyle = col; ctx.fillText(`${lab} ${rr.toFixed(1)}m`, cx + 3, cy - rr * sc);
        }
        for (let i = 0; i < acc.pts.length; i++) {
          const p = acc.pts[i]!;
          const fade = 0.25 + 0.75 * (i / Math.max(1, acc.pts.length - 1));
          ctx.fillStyle = `rgba(54,211,153,${fade.toFixed(2)})`;
          ctx.beginPath(); ctx.arc(cx + p.e * sc, cy - p.n * sc, i === acc.pts.length - 1 ? 4 : 2, 0, Math.PI * 2); ctx.fill();
        }
      }} />
      <dl className="kv compact stat3">
        <Row k="CEP 50%" v={acc.n >= 4 ? `${acc.cep.toFixed(2)} m` : "—"} />
        <Row k="R95 95%" v={acc.n >= 4 ? `${acc.r95.toFixed(2)} m` : "—"} />
        <Row k="2DRMS" v={acc.n >= 4 ? `${acc.twodrms.toFixed(2)} m` : "—"} />
        <Row k="σ East" v={`${acc.sE.toFixed(2)} m`} />
        <Row k="σ North" v={`${acc.sN.toFixed(2)} m`} />
        <Row k="σ Alt" v={`${acc.sAlt.toFixed(2)} m`} />
        <Row k="HDOP" v={f.hdop != null ? f.hdop.toFixed(2) : "—"} />
        <Row k="VDOP" v={f.vdop != null ? f.vdop.toFixed(2) : "—"} />
        <Row k="PDOP" v={f.pdop != null ? f.pdop.toFixed(2) : "—"} />
      </dl>
    </section>
  );
}

// ---------- PPS / time precision ----------
export function TimingPanel({ s, timing }: { s: GnssState; timing: Timing }) {
  const dev = s.ppsDev;
  return (
    <section className="panel">
      <h2>PPS / time precision <span className="hdr-aux">{timing.n} samples</span></h2>
      <div className="chart-label">interval deviation (µs) · history</div>
      <Canvas className="canvas-line" draw={(ctx, w, h) => {
        if (dev.length < 2) return;
        const maxAbs = Math.max(20, ...dev.map((d) => Math.abs(d)));
        ctx.strokeStyle = "#22303d"; ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();
        ctx.fillStyle = "#475569"; ctx.font = "9px ui-monospace, monospace"; ctx.textAlign = "left";
        ctx.fillText(`+${maxAbs.toFixed(0)}`, 2, 9); ctx.fillText(`-${maxAbs.toFixed(0)}`, 2, h - 3);
        drawLine(ctx, w, h, dev.slice(-200), "#36d399", -maxAbs, maxAbs, 3);
      }} />
      <div className="chart-label">jitter histogram</div>
      <Canvas className="canvas-line" draw={(ctx, w, h) => {
        if (dev.length < 4) return;
        const lo = Math.min(...dev), hi = Math.max(...dev), span = Math.max(1, hi - lo);
        const bins = 24, counts = new Array(bins).fill(0);
        for (const d of dev) counts[Math.min(bins - 1, Math.floor(((d - lo) / span) * bins))]++;
        const mx = Math.max(...counts);
        const bw = w / bins;
        for (let i = 0; i < bins; i++) {
          const bh = (h - 12) * (counts[i] / mx);
          ctx.fillStyle = "#3b82f6"; ctx.fillRect(i * bw + 0.5, h - 12 - bh, bw - 1, bh);
        }
        ctx.fillStyle = "#475569"; ctx.font = "9px ui-monospace, monospace";
        ctx.textAlign = "left"; ctx.fillText(`${lo.toFixed(0)}µs`, 1, h - 2);
        ctx.textAlign = "right"; ctx.fillText(`${hi.toFixed(0)}µs`, w - 1, h - 2);
      }} />
      <dl className="kv compact stat3">
        <Row k="interval µ" v={timing.n >= 2 ? `${timing.meanInterval.toFixed(1)}` : "—"} />
        <Row k="jitter σ" v={timing.n >= 2 ? `${timing.sigma.toFixed(2)} µs` : "—"} />
        <Row k="peak-peak" v={timing.n >= 2 ? `${timing.pp.toFixed(0)} µs` : "—"} />
        <Row k="osc offset" v={timing.n >= 2 ? `${timing.ppm >= 0 ? "+" : ""}${timing.ppm.toFixed(2)} ppm` : "—"} />
        <Row k="est. UTC σ" v={timing.n >= 5 ? `±${timing.sigma.toFixed(1)} µs` : "—"} />
        <Row k="missed" v={s.pps?.missed ?? 0} />
      </dl>
    </section>
  );
}

// ---------- Time series ----------
export function TimeSeries({ s }: { s: GnssState }) {
  const t = s.track;
  const alt = t.map((x) => x.alt);
  const used = t.map((x) => x.satsUsed);
  const inv = t.map((x) => x.inView);
  const spd = t.map((x) => x.speedKmh);
  return (
    <section className="panel">
      <h2>Time series <span className="hdr-aux">last ~{Math.min(10, Math.ceil(t.length / 60))} min</span></h2>
      <div className="chart-label">altitude (m)</div>
      <Canvas className="canvas-mini" draw={(ctx, w, h) => {
        const [lo, hi] = rangeOf([alt], [0, 100]); drawLine(ctx, w, h, alt, "#38bdf8", lo, hi);
        labelLastFirst(ctx, w, h, alt, (v) => v.toFixed(0));
      }} />
      <div className="chart-label">satellites · used (green) / in view (gray)</div>
      <Canvas className="canvas-mini" draw={(ctx, w, h) => {
        const [lo, hi] = rangeOf([used, inv], [0, 12]);
        drawLine(ctx, w, h, inv, "#64748b", lo, hi); drawLine(ctx, w, h, used, "#36d399", lo, hi);
        labelLastFirst(ctx, w, h, used, (v) => v.toFixed(0));
      }} />
      <div className="chart-label">speed (km/h)</div>
      <Canvas className="canvas-mini" draw={(ctx, w, h) => {
        const [lo, hi] = rangeOf([spd], [0, 5]); drawLine(ctx, w, h, spd, "#fbbd23", lo, hi);
        labelLastFirst(ctx, w, h, spd, (v) => v.toFixed(1));
      }} />
    </section>
  );
}

function labelLastFirst(ctx: CanvasRenderingContext2D, w: number, _h: number, data: (number | null)[], fmt: (v: number) => string) {
  const last = [...data].reverse().find((x): x is number => x != null);
  ctx.fillStyle = "#7a8a9a"; ctx.font = "9px ui-monospace, monospace"; ctx.textAlign = "right"; ctx.textBaseline = "top";
  if (last != null) ctx.fillText(fmt(last), w - 2, 1);
}

// 共有: kv 行
export function Row({ k, v }: { k: string; v: React.ReactNode }) {
  return (
    <div>
      <dt>{k}</dt>
      <dd>{v}</dd>
    </div>
  );
}
