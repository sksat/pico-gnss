import { useEffect, useRef, useState } from "react";
import { Canvas } from "./Canvas";
import { Row, drawSpark } from "./charts";
import { snrColor, sysColor, SYS_INFO } from "../nmea";
import type { Accuracy, Timing } from "../stats";
import type { GnssState, Sync } from "../types";

const QUALITY = ["No fix", "GPS", "DGPS", "PPS", "RTK", "Float RTK", "Estimated", "Manual", "Sim"];
const p2 = (n: number) => String(n).padStart(2, "0");
const p3 = (n: number) => String(n).padStart(3, "0");
const signed = (n: number) => (n >= 0 ? "+" : "") + n;
function dms(dd: number, lat: boolean): string {
  const hemi = lat ? (dd >= 0 ? "N" : "S") : dd >= 0 ? "E" : "W";
  const a = Math.abs(dd), d = Math.floor(a);
  return `${d}°${((a - d) * 60).toFixed(4)}′${hemi}`;
}

function Clock({ sync }: { sync: Sync | null }) {
  const [txt, setTxt] = useState("--:--:--");
  useEffect(() => {
    let raf = 0;
    const tick = () => {
      if (sync) {
        const d = new Date(sync.unix_s * 1000 + (Date.now() - sync.wall));
        setTxt(`${p2(d.getUTCHours())}:${p2(d.getUTCMinutes())}:${p2(d.getUTCSeconds())}.${p3(d.getUTCMilliseconds())}`);
      }
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [sync]);
  return <div className="clock-time">{txt}</div>;
}

export function Header({ s, acc, timing }: { s: GnssState; acc: Accuracy; timing: Timing }) {
  return (
    <header className="topbar">
      <div className="brand">
        <span className="logo">◈</span>
        <span className="title">pico-gnss</span>
        <span className="subtitle">GYSFFMANC · RP2040</span>
      </div>
      <div className="metrics">
        <div className="metric">
          <div className="metric-val">{acc.n >= 8 ? acc.twodrms.toFixed(2) : "—"} <span className="u">m</span></div>
          <div className="metric-lbl">horizontal (2DRMS 95%)</div>
        </div>
        <div className="metric">
          <div className="metric-val">{timing.n >= 5 ? "±" + timing.sigma.toFixed(1) : "—"} <span className="u">µs</span></div>
          <div className="metric-lbl">time sync (1σ jitter)</div>
        </div>
      </div>
      <div className="clock">
        <Clock sync={s.sync} />
        <div className="clock-label">
          GNSS-disciplined UTC
          <span className={"lock-badge" + (s.sync ? " locked" : "")}>{s.sync ? "PPS LOCK" : "NO SYNC"}</span>
        </div>
      </div>
      <div className={"conn" + (s.conn.up ? " up" : "")}>
        <span className="dot" />
        <span>{s.conn.text}</span>
        <span className="src">{s.conn.src}</span>
      </div>
    </header>
  );
}

export function FixPanel({ s }: { s: GnssState }) {
  const f = s.fix;
  const cls = f.quality > 0 ? (f.quality >= 2 ? "dgps" : "ok") : "";
  return (
    <section className="panel a-fix">
      <h2>Fix</h2>
      <div className={"fix-status " + cls}>{f.quality > 0 ? (QUALITY[f.quality] ?? "FIX").toUpperCase() : "NO FIX"}</div>
      <dl className="kv">
        <Row k="Lat" v={f.lat != null ? `${f.lat.toFixed(6)}  ${dms(f.lat, true)}` : "—"} />
        <Row k="Lon" v={f.lon != null ? `${f.lon.toFixed(6)}  ${dms(f.lon, false)}` : "—"} />
        <Row k="Alt" v={f.alt != null ? `${f.alt.toFixed(1)} m` : "—"} />
        <Row k="Mode" v={f.mode === 3 ? "3D fix" : f.mode === 2 ? "2D fix" : "—"} />
        <Row k="Sats used" v={f.satsUsed ?? "—"} />
        <Row k="Speed" v={f.speedKn != null ? `${(f.speedKn * 1.852).toFixed(1)} km/h` : "—"} />
        <Row k="Course" v={f.courseDeg != null ? `${f.courseDeg.toFixed(0)}°` : "—"} />
      </dl>
    </section>
  );
}

export function PpsPanel({ s }: { s: GnssState }) {
  const p = s.pps;
  return (
    <section className="panel a-pps">
      <h2>PPS</h2>
      <div className="pps-big">
        <span>{p && p.interval_us > 0 ? p.interval_us.toLocaleString() : "—"}</span>
        <span className="unit">µs</span>
      </div>
      <div className="pps-dev">dev {p && p.interval_us > 0 ? signed(p.dev) : "—"} µs · {p?.state ?? "—"}</div>
      <Canvas className="spark" draw={(ctx, w, h) => drawSpark(ctx, w, h, s.ppsDev)} />
      <dl className="kv compact">
        <Row k="count" v={p?.count ?? 0} />
        <Row k="missed" v={p?.missed ?? 0} />
        <Row k="osc drift" v={s.sync ? `${signed(s.sync.drift_us)} µs` : "—"} />
      </dl>
    </section>
  );
}

export function SyncPanel({ s }: { s: GnssState }) {
  const y = s.sync;
  const iso = y ? new Date(y.unix_s * 1000).toISOString().replace(".000Z", "Z").replace("T", " ") : "—";
  return (
    <section className="panel a-sync">
      <h2>Time sync</h2>
      <div className="sync-utc">{iso}</div>
      <dl className="kv compact">
        <Row k="unix_s" v={y?.unix_s ?? "—"} />
        <Row k="pps local" v={y ? `${(y.pps_local_us / 1e6).toFixed(3)} s` : "—"} />
        <Row k="osc drift" v={y ? `${signed(y.drift_us)} µs` : "—"} />
      </dl>
      <p className="hint">PPS エッジ↔UTC 秒の対応付けは firmware (RP2040, 1µs) 側で実施。host 同期のジッタ (数十ms) を避ける。</p>
    </section>
  );
}

export function SatTable({ s }: { s: GnssState }) {
  const sats = [...s.sats].sort((a, b) => (a.sys === b.sys ? a.prn - b.prn : a.sys.localeCompare(b.sys)));
  const bySys = new Map<string, number>();
  for (const x of s.sats) bySys.set(x.sys, (bySys.get(x.sys) ?? 0) + 1);
  const summary = [...bySys.entries()].map(([k, v]) => `${SYS_INFO[k]?.name ?? k}:${v}`).join(" · ");
  return (
    <section className="panel">
      <h2>Satellites <span className="hdr-aux">{summary || "—"} · {sats.length} in view</span></h2>
      <div className="table-wrap">
        <table className="sat-table">
          <thead>
            <tr><th>Sys</th><th>PRN</th><th>Elev</th><th>Azim</th><th>C/N₀</th><th className="bar-col">signal</th><th>Used</th></tr>
          </thead>
          <tbody>
            {sats.map((x) => (
              <tr key={x.sys + x.prn} className={s.usedPrn.has(x.prn) ? "used" : ""}>
                <td><span className="sys-tag" style={{ background: sysColor(x.sys) }}>{SYS_INFO[x.sys]?.name ?? x.sys}</span></td>
                <td>{x.prn}</td>
                <td>{x.elev != null ? `${x.elev}°` : "—"}</td>
                <td>{x.azim != null ? `${x.azim}°` : "—"}</td>
                <td>{x.snr != null && x.snr > 0 ? x.snr : "—"}</td>
                <td className="bar-col"><div className="snr-bar" style={{ width: `${Math.min(100, ((x.snr ?? 0) / 55) * 100)}%`, background: snrColor(x.snr) }} /></td>
                <td className="used-dot">{s.usedPrn.has(x.prn) ? "●" : ""}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </section>
  );
}

export function ConsolePanel({ s }: { s: GnssState }) {
  const ref = useRef<HTMLDivElement>(null);
  const atBottom = useRef(true);
  useEffect(() => {
    const el = ref.current;
    if (el && atBottom.current) el.scrollTop = el.scrollHeight;
  });
  return (
    <section className="panel">
      <h2>NMEA stream<span className="resize-hint">⤡ ドラッグで高さ調整</span></h2>
      <div
        className="console"
        ref={ref}
        onScroll={(e) => {
          const el = e.currentTarget;
          atBottom.current = el.scrollHeight - el.scrollTop - el.clientHeight < 30;
        }}
      >
        {s.raw.map((l, i) => (
          <div className="ln" key={i}>
            <span className="ts">{l.ts}</span>
            <span className={"t-" + l.kind}>{l.sentence}</span>
          </div>
        ))}
      </div>
    </section>
  );
}
