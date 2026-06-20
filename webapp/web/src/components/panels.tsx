import { useEffect, useRef, useState } from "react";
import { Canvas } from "./Canvas";
import { Row, drawSpark } from "./charts";
import { snrColor, sysColor, SYS_INFO } from "../nmea";
import { assessReception } from "../stats";
import type { Accuracy, Timing } from "../stats";
import type { GnssState } from "../types";

const QUALITY = ["No fix", "GPS", "DGPS", "PPS", "RTK", "Float RTK", "Estimated", "Manual", "Sim"];
const p2 = (n: number) => String(n).padStart(2, "0");
const p3 = (n: number) => String(n).padStart(3, "0");
const signed = (n: number) => (n >= 0 ? "+" : "") + n;
/** ns のジッタ (常に正, σ) を読みやすい単位で。 */
function jitterStr(ns: number): string {
  return Math.abs(ns) < 1000 ? `±${ns.toFixed(1)} ns` : `±${(ns / 1000).toFixed(2)} µs`;
}
/** 符号付きの誤差 (ns) を読みやすい単位で。 */
function errStr(ns: number): string {
  const s = ns >= 0 ? "+" : "";
  return Math.abs(ns) < 1000 ? `${s}${ns} ns` : `${s}${(ns / 1000).toFixed(2)} µs`;
}
function dms(dd: number, lat: boolean): string {
  const hemi = lat ? (dd >= 0 ? "N" : "S") : dd >= 0 ? "E" : "W";
  const a = Math.abs(dd), d = Math.floor(a);
  return `${d}°${((a - d) * 60).toFixed(4)}′${hemi}`;
}

/** epochMs(受信時の UTC ms) + 経過 を rAF で進める。GPSDO の規律 UTC を優先。 */
function Clock({ epochMs, wall }: { epochMs: number | null; wall: number }) {
  const [txt, setTxt] = useState("--:--:--");
  useEffect(() => {
    let raf = 0;
    const tick = () => {
      if (epochMs != null) {
        const d = new Date(epochMs + (Date.now() - wall));
        setTxt(`${p2(d.getUTCHours())}:${p2(d.getUTCMinutes())}:${p2(d.getUTCSeconds())}.${p3(d.getUTCMilliseconds())}`);
      }
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [epochMs, wall]);
  return <div className="clock-time">{txt}</div>;
}

export function Header({ s, acc, timing }: { s: GnssState; acc: Accuracy; timing: Timing }) {
  return (
    <header className="topbar">
      <div className="brand">
        <span className="logo">◈</span>
        <span className="title">pico-gnss</span>
        <span className="subtitle">{s.fw ? `${s.fw} · RP2040` : "GYSFFMANC · RP2040"}</span>
      </div>
      <div className="metrics">
        <div className="metric">
          <div className="metric-val">{acc.n >= 8 ? acc.cep.toFixed(2) : "—"} <span className="u">m</span></div>
          <div className="metric-lbl">horizontal CEP 50% · R95 {acc.n >= 8 ? acc.r95.toFixed(1) : "—"}m</div>
        </div>
        <div className="metric">
          <div className="metric-val">{timing.n >= 5 ? jitterStr(timing.sigma) : "—"}</div>
          <div className="metric-lbl">PPS jitter (1σ · PIO 16ns capture)</div>
        </div>
      </div>
      <div className="clock">
        <Clock
          epochMs={s.gpsdo ? s.gpsdo.unixMs : s.sync ? s.sync.unix_s * 1000 : null}
          wall={s.gpsdo ? s.gpsdo.wall : s.sync ? s.sync.wall : 0}
        />
        <div className="clock-label">
          GPSDO-disciplined UTC
          <span className={"lock-badge" + (s.gpsdo?.locked ? " locked" : "")}>
            {s.gpsdo ? (s.gpsdo.locked ? "GPSDO LOCK" : "ACQUIRING") : "NO SYNC"}
          </span>
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

export function ReceptionPanel({ s, timing }: { s: GnssState; timing: Timing }) {
  const r = assessReception(s, timing);
  return (
    <section className="panel a-rec">
      <h2>Reception <span className="hdr-aux">timing quality ceiling</span></h2>
      <div className={"fix-status " + r.cls}>{r.verdict}</div>
      <dl className="kv compact">
        {r.factors.map((x) => (
          <Row key={x.k} k={x.k} v={<span style={{ color: x.color }}>{x.text}</span>} />
        ))}
      </dl>
      <p className="hint">受信条件 (衛星数・幾何 HDOP・信号 C/N₀) が PPS タイミング σ の上限を決める。緑=good / 橙=fair / 赤=poor。</p>
    </section>
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
      <div className="pps-dev">dev {p && p.interval_ns > 0 ? signed(p.dev) : "—"} ns · {p?.state ?? "—"}</div>
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
  const g = s.gpsdo;
  const y = s.sync;
  const iso = y ? new Date(y.unix_s * 1000).toISOString().replace(".000Z", "Z").replace("T", " ") : "—";
  const inHoldover = g != null && g.holdoverMs > 2000;
  return (
    <section className="panel a-sync">
      <h2>GPSDO <span className="hdr-aux">{g ? (g.locked ? "locked" : "acquiring") : "—"}</span></h2>
      <div className="sync-utc">{iso}</div>
      <dl className="kv compact">
        <Row k="disciplined freq" v={g ? `${g.ppb >= 0 ? "+" : ""}${(g.ppb / 1000).toFixed(3)} ppm` : "—"} />
        <Row k="clock err (corr)" v={y ? errStr(y.err_ns) : "—"} />
        <Row k="drift removed" v={g ? `~${(Math.abs(g.ppb) / 1000).toFixed(1)} µs/s` : "—"} />
        <Row k="holdover" v={g ? (inHoldover ? `${(g.holdoverMs / 1000).toFixed(0)} s ⚠` : "PPS locked") : "—"} />
        <Row k="PPS gen (GP3→4)" v={s.ppsGen ? `jitter ${s.ppsGen.jitter_ns} ns (PIO)` : "—"} />
        <Row k="PPS gen phase" v={s.ppsGen ? `${Math.abs(s.ppsGen.phase_ns) < 1000 ? s.ppsGen.phase_ns + " ns" : (s.ppsGen.phase_ns / 1e6).toFixed(2) + " ms"} ← UTC` : "—"} />
      </dl>
      <p className="hint">PIO の ns 精度 PPS 間隔で水晶ドリフトを推定し UTC を規律。clock err = 補正後の 1 秒先読み残差
        (この精度で時刻を保持; PPS 断中も周波数外挿)。drift removed = 補正しなければ毎秒ずれていた量。</p>
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
