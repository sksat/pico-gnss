/**
 * pico-gnss realtime dashboard (browser side, TypeScript / ES2024).
 *
 * server.ts から WebSocket で届く {nmea|pps|sync|status} を受け取り、NMEA をパースして
 * 地図・スカイプロット・C/N0・PPS・時刻同期・生ストリームを描画する。
 *
 * Leaflet は CDN のグローバル `L` を使う (型は @types/leaflet の UMD global)。
 */

// ---- server からのメッセージ ----
type Msg =
  | { t: "nmea"; s: string }
  | { t: "pps"; count: number; interval_us: number; state: string; missed: number }
  | { t: "sync"; pps_local_us: number; unix_s: number; drift_us: number }
  | { t: "status"; source: string; connected: boolean; note?: string };

// ---- GNSS 状態 ----
interface Sat {
  sys: string; // GP/GL/GA/GB/GQ/...
  prn: number;
  elev: number | null;
  azim: number | null;
  snr: number | null;
  seen: number; // performance.now()
}
interface Fix {
  lat: number | null;
  lon: number | null;
  alt: number | null;
  satsUsed: number | null;
  hdop: number | null;
  quality: number;
  speedKn: number | null;
  courseDeg: number | null;
  status: string;
}

const sats = new Map<string, Sat>(); // key = sys+prn
const usedPrn = new Map<number, number>(); // prn -> lastSeen
const fix: Fix = {
  lat: null, lon: null, alt: null, satsUsed: null,
  hdop: null, quality: 0, speedKn: null, courseDeg: null, status: "V",
};
let lastSync: { unix_s: number; pps_local_us: number; drift_us: number; wall: number } | null = null;
const ppsDev: number[] = []; // 直近の (interval-1e6)
let nmeaCount = 0;
let nmeaWindowStart = performance.now();
let nmeaRate = 0;

const SAT_STALE_MS = 8000;

const CONSTELLATION: Record<string, { name: string; color: string }> = {
  GP: { name: "GPS", color: "#38bdf8" },
  GL: { name: "GLONASS", color: "#f472b6" },
  GA: { name: "Galileo", color: "#a78bfa" },
  GB: { name: "BeiDou", color: "#fbbf24" },
  BD: { name: "BeiDou", color: "#fbbf24" },
  GQ: { name: "QZSS", color: "#34d399" },
  GN: { name: "GNSS", color: "#94a3b8" },
};
function sysColor(sys: string): string {
  return CONSTELLATION[sys]?.color ?? "#94a3b8";
}
function snrColor(snr: number | null): string {
  if (snr == null) return "#3a4654";
  if (snr < 20) return "#f87272";
  if (snr < 30) return "#fbbd23";
  if (snr < 38) return "#a3e635";
  return "#36d399";
}

// ---- NMEA parsing ----
function field(parts: string[], i: number): string {
  return parts[i] ?? "";
}
function num(s: string): number | null {
  if (s === "") return null;
  const v = Number(s);
  return Number.isFinite(v) ? v : null;
}
function nmeaCoord(val: string, hemi: string): number | null {
  if (!val) return null;
  const dot = val.indexOf(".");
  if (dot < 3) return null;
  const degLen = dot - 2;
  const deg = parseInt(val.slice(0, degLen), 10);
  const min = parseFloat(val.slice(degLen));
  if (Number.isNaN(deg) || Number.isNaN(min)) return null;
  let dd = deg + min / 60;
  if (hemi === "S" || hemi === "W") dd = -dd;
  return dd;
}

function handleNmea(sentence: string): void {
  nmeaCount++;
  const star = sentence.indexOf("*");
  const body = star >= 0 ? sentence.slice(0, star) : sentence;
  const parts = body.split(",");
  const type0 = parts[0] ?? "";
  const sys = type0.slice(1, 3); // talker
  const kind = type0.slice(3); // GGA/RMC/...
  const now = performance.now();

  switch (kind) {
    case "GGA": {
      const lat = nmeaCoord(field(parts, 2), field(parts, 3));
      const lon = nmeaCoord(field(parts, 4), field(parts, 5));
      fix.quality = num(field(parts, 6)) ?? 0;
      fix.satsUsed = num(field(parts, 7));
      fix.hdop = num(field(parts, 8));
      fix.alt = num(field(parts, 9));
      if (fix.quality > 0 && lat != null && lon != null) {
        fix.lat = lat;
        fix.lon = lon;
        onPosition(lat, lon);
      } else if (fix.quality === 0) {
        fix.lat = null;
        fix.lon = null;
      }
      break;
    }
    case "RMC": {
      fix.status = field(parts, 2);
      fix.speedKn = num(field(parts, 7));
      fix.courseDeg = num(field(parts, 8));
      const lat = nmeaCoord(field(parts, 3), field(parts, 4));
      const lon = nmeaCoord(field(parts, 5), field(parts, 6));
      if (fix.status === "A" && lat != null && lon != null) {
        fix.lat = lat;
        fix.lon = lon;
        onPosition(lat, lon);
      }
      break;
    }
    case "GSV": {
      // parts: type,totalMsg,msgNum,inView, then groups of (prn,elev,azim,snr)
      for (let i = 4; i + 3 < parts.length; i += 4) {
        const prn = num(field(parts, i));
        if (prn == null) continue;
        const sat: Sat = {
          sys,
          prn,
          elev: num(field(parts, i + 1)),
          azim: num(field(parts, i + 2)),
          snr: num(field(parts, i + 3)),
          seen: now,
        };
        sats.set(sys + prn, sat);
      }
      break;
    }
    case "GSA": {
      // parts[3..14] = PRNs used in fix
      for (let i = 3; i <= 14; i++) {
        const prn = num(field(parts, i));
        if (prn != null) usedPrn.set(prn, now);
      }
      break;
    }
    case "VTG": {
      fix.courseDeg = num(field(parts, 1)) ?? fix.courseDeg;
      fix.speedKn = num(field(parts, 5)) ?? fix.speedKn;
      break;
    }
    default:
      break;
  }
  appendConsole(sentence, kind);
  scheduleRender();
}

function pruneStale(): void {
  const now = performance.now();
  for (const [k, s] of sats) if (now - s.seen > SAT_STALE_MS) sats.delete(k);
  for (const [p, t] of usedPrn) if (now - t > SAT_STALE_MS) usedPrn.delete(p);
}

// ===================== DOM =====================
const $ = (id: string) => document.getElementById(id)!;

// ---- console ----
const consoleEl = $("console");
function appendConsole(sentence: string, kind: string): void {
  const div = document.createElement("div");
  div.className = "ln";
  const d = new Date();
  const ts = `${String(d.getHours()).padStart(2, "0")}:${String(d.getMinutes()).padStart(2, "0")}:${String(d.getSeconds()).padStart(2, "0")}`;
  div.innerHTML = `<span class="ts">${ts}</span><span class="t-${kind}">${escapeHtml(sentence)}</span>`;
  const atBottom = consoleEl.scrollHeight - consoleEl.scrollTop - consoleEl.clientHeight < 30;
  consoleEl.appendChild(div);
  while (consoleEl.childElementCount > 250) consoleEl.removeChild(consoleEl.firstChild!);
  if (atBottom) consoleEl.scrollTop = consoleEl.scrollHeight;
}
function escapeHtml(s: string): string {
  return s.replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" })[c]!);
}

// ---- map ----
let map: import("leaflet").Map | null = null;
let marker: import("leaflet").CircleMarker | null = null;
let trail: import("leaflet").Polyline | null = null;
const trailPts: [number, number][] = [];
let mapCentered = false;

function initMap(): void {
  if (typeof L === "undefined") return;
  map = L.map("map", { zoomControl: true, attributionControl: true }).setView([35.681236, 139.767125], 16);
  L.tileLayer("https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png", {
    maxZoom: 20,
    attribution: "© OpenStreetMap · © CARTO",
  }).addTo(map);
  trail = L.polyline([], { color: "#36d399", weight: 2, opacity: 0.8 }).addTo(map);
  marker = L.circleMarker([35.681236, 139.767125], {
    radius: 7, color: "#36d399", weight: 2, fillColor: "#36d399", fillOpacity: 0.5,
  }).addTo(map);
}
function onPosition(lat: number, lon: number): void {
  if (!map || !marker || !trail) return;
  marker.setLatLng([lat, lon]);
  const last = trailPts[trailPts.length - 1];
  if (!last || last[0] !== lat || last[1] !== lon) {
    trailPts.push([lat, lon]);
    if (trailPts.length > 3000) trailPts.shift();
    trail.setLatLngs(trailPts);
  }
  if (!mapCentered) {
    map.setView([lat, lon], 17);
    mapCentered = true;
  } else {
    map.panTo([lat, lon], { animate: true, duration: 0.5 });
  }
  $("map-coords").textContent = `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
}

// ---- canvas helpers ----
function setupCanvas(c: HTMLCanvasElement): { ctx: CanvasRenderingContext2D; w: number; h: number } {
  const dpr = window.devicePixelRatio || 1;
  const rect = c.getBoundingClientRect();
  const w = Math.max(1, Math.round(rect.width));
  const h = Math.max(1, Math.round(rect.height));
  if (c.width !== w * dpr || c.height !== h * dpr) {
    c.width = w * dpr;
    c.height = h * dpr;
  }
  const ctx = c.getContext("2d")!;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

// ---- skyplot ----
function drawSkyplot(): void {
  const c = $("skyplot") as HTMLCanvasElement;
  const { ctx, w, h } = setupCanvas(c);
  ctx.clearRect(0, 0, w, h);
  const cx = w / 2;
  const cy = h / 2;
  const R = Math.min(w, h) / 2 - 16;

  // rings (elev 0/30/60) + cardinal
  ctx.strokeStyle = "#1f2a36";
  ctx.fillStyle = "#5a6b7b";
  ctx.font = "10px ui-monospace, monospace";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  for (const elev of [0, 30, 60]) {
    const r = (R * (90 - elev)) / 90;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.beginPath();
  ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R);
  ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
  ctx.stroke();
  for (const [lbl, dx, dy] of [["N", 0, -R - 8], ["S", 0, R + 8], ["E", R + 8, 0], ["W", -R - 8, 0]] as const) {
    ctx.fillText(lbl, cx + dx, cy + dy);
  }

  let shown = 0;
  for (const s of sats.values()) {
    if (s.elev == null || s.azim == null) continue;
    const r = (R * (90 - Math.max(0, Math.min(90, s.elev)))) / 90;
    const a = (s.azim * Math.PI) / 180;
    const x = cx + r * Math.sin(a);
    const y = cy - r * Math.cos(a);
    const tracked = s.snr != null && s.snr > 0;
    ctx.beginPath();
    ctx.arc(x, y, tracked ? 7 : 4, 0, Math.PI * 2);
    ctx.fillStyle = tracked ? snrColor(s.snr) : "#33414f";
    ctx.fill();
    if (usedPrn.has(s.prn)) {
      ctx.lineWidth = 2;
      ctx.strokeStyle = sysColor(s.sys);
      ctx.stroke();
    }
    if (tracked) {
      ctx.fillStyle = "#0b0f14";
      ctx.font = "9px ui-monospace, monospace";
      ctx.fillText(String(s.prn), x, y);
      shown++;
    }
  }
  $("sky-count").textContent = `${sats.size} in view · ${shown} tracked`;

  // legend
  const legend = $("constellation-legend");
  const present = new Set<string>();
  for (const s of sats.values()) present.add(s.sys);
  legend.innerHTML = "";
  for (const sysk of present) {
    const c2 = CONSTELLATION[sysk];
    const span = document.createElement("span");
    span.innerHTML = `<i style="background:${sysColor(sysk)}"></i>${c2?.name ?? sysk}`;
    legend.appendChild(span);
  }
}

// ---- SNR bar chart ----
function drawSnr(): void {
  const c = $("snr") as HTMLCanvasElement;
  const { ctx, w, h } = setupCanvas(c);
  ctx.clearRect(0, 0, w, h);
  const list = [...sats.values()]
    .filter((s) => s.snr != null && s.snr > 0)
    .sort((a, b) => (a.sys === b.sys ? a.prn - b.prn : a.sys.localeCompare(b.sys)));
  $("snr-aux").textContent = `${list.length} tracked`;
  const padB = 26;
  const padT = 8;
  const maxSnr = 55;
  const n = list.length || 1;
  const bw = Math.min(34, (w - 8) / n);
  const x0 = 4;

  // grid lines (20/30/40 dBHz)
  ctx.strokeStyle = "#1a2530";
  ctx.fillStyle = "#475569";
  ctx.font = "9px ui-monospace, monospace";
  ctx.textAlign = "left";
  for (const g of [20, 30, 40, 50]) {
    const y = padT + (h - padT - padB) * (1 - g / maxSnr);
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
    ctx.fillText(String(g), 2, y - 2);
  }

  list.forEach((s, i) => {
    const snr = s.snr!;
    const bh = (h - padT - padB) * Math.min(1, snr / maxSnr);
    const x = x0 + i * bw;
    const y = h - padB - bh;
    ctx.fillStyle = snrColor(snr);
    ctx.fillRect(x + 1, y, bw - 2, bh);
    // used marker
    if (usedPrn.has(s.prn)) {
      ctx.fillStyle = sysColor(s.sys);
      ctx.fillRect(x + 1, h - padB - bh - 3, bw - 2, 2);
    }
    // prn label
    ctx.save();
    ctx.translate(x + bw / 2, h - padB + 3);
    ctx.rotate(-Math.PI / 2);
    ctx.fillStyle = "#7a8a9a";
    ctx.font = "9px ui-monospace, monospace";
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    ctx.fillText(`${s.sys}${s.prn}`, 0, 0);
    ctx.restore();
  });
}

// ---- pps sparkline ----
function drawSpark(): void {
  const c = $("pps-spark") as HTMLCanvasElement;
  const { ctx, w, h } = setupCanvas(c);
  ctx.clearRect(0, 0, w, h);
  if (ppsDev.length < 2) return;
  const data = ppsDev.slice(-80);
  const maxAbs = Math.max(50, ...data.map((d) => Math.abs(d)));
  const mid = h / 2;
  ctx.strokeStyle = "#22303d";
  ctx.beginPath(); ctx.moveTo(0, mid); ctx.lineTo(w, mid); ctx.stroke();
  ctx.strokeStyle = "#36d399";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  data.forEach((d, i) => {
    const x = (i / (data.length - 1)) * w;
    const y = mid - (d / maxAbs) * (h / 2 - 3);
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.stroke();
}

// ---- fix / pps / sync panels ----
function toDMS(dd: number, lat: boolean): string {
  const h = lat ? (dd >= 0 ? "N" : "S") : dd >= 0 ? "E" : "W";
  const a = Math.abs(dd);
  const d = Math.floor(a);
  const m = (a - d) * 60;
  return `${d}°${m.toFixed(4)}′${h}`;
}
const QUALITY = ["No fix", "GPS", "DGPS", "PPS", "RTK", "Float RTK", "Estimated"];

function renderFix(): void {
  const st = $("fix-status");
  if (fix.quality > 0) {
    st.textContent = (QUALITY[fix.quality] ?? "FIX").toUpperCase();
    st.className = "fix-status " + (fix.quality >= 2 ? "dgps" : "ok");
  } else {
    st.textContent = "NO FIX";
    st.className = "fix-status";
  }
  $("f-lat").textContent = fix.lat != null ? `${fix.lat.toFixed(6)}  ${toDMS(fix.lat, true)}` : "—";
  $("f-lon").textContent = fix.lon != null ? `${fix.lon.toFixed(6)}  ${toDMS(fix.lon, false)}` : "—";
  $("f-alt").textContent = fix.alt != null ? `${fix.alt.toFixed(1)} m` : "—";
  $("f-sats").textContent = fix.satsUsed != null ? String(fix.satsUsed) : "—";
  $("f-hdop").textContent = fix.hdop != null ? fix.hdop.toFixed(2) : "—";
  $("f-speed").textContent = fix.speedKn != null ? `${(fix.speedKn * 1.852).toFixed(1)} km/h` : "—";
  $("f-course").textContent = fix.courseDeg != null ? `${fix.courseDeg.toFixed(0)}°` : "—";
  $("f-qual").textContent = `${QUALITY[fix.quality] ?? "?"} (${fix.quality})`;
}

function onPps(m: Extract<Msg, { t: "pps" }>): void {
  if (m.interval_us > 0) {
    const dev = m.interval_us - 1_000_000;
    ppsDev.push(dev);
    if (ppsDev.length > 300) ppsDev.shift();
    $("pps-interval").textContent = m.interval_us.toLocaleString();
    $("pps-dev").textContent = (dev >= 0 ? "+" : "") + dev;
  }
  $("pps-state").textContent = m.state;
  $("pps-count").textContent = String(m.count);
  $("pps-missed").textContent = String(m.missed);
  drawSpark();
}

function onSync(m: Extract<Msg, { t: "sync" }>): void {
  lastSync = { unix_s: m.unix_s, pps_local_us: m.pps_local_us, drift_us: m.drift_us, wall: Date.now() };
  $("sync-unix").textContent = String(m.unix_s);
  $("sync-local").textContent = `${(m.pps_local_us / 1e6).toFixed(3)} s`;
  const drift = `${m.drift_us >= 0 ? "+" : ""}${m.drift_us} µs`;
  $("sync-drift").textContent = drift;
  $("pps-drift").textContent = drift;
  $("sync-utc").textContent = isoFromUnix(m.unix_s) + "Z";
  const badge = $("lock-badge");
  badge.textContent = "PPS LOCK";
  badge.classList.add("locked");
}

function isoFromUnix(unixS: number): string {
  return new Date(unixS * 1000).toISOString().replace(".000Z", "").replace("T", " ");
}

// ---- header clock (PPS-disciplined second + host-interpolated subsecond) ----
function tickClock(): void {
  const el = $("utc-time");
  if (lastSync) {
    const ms = lastSync.unix_s * 1000 + (Date.now() - lastSync.wall);
    const d = new Date(ms);
    el.textContent =
      `${String(d.getUTCHours()).padStart(2, "0")}:${String(d.getUTCMinutes()).padStart(2, "0")}:${String(d.getUTCSeconds()).padStart(2, "0")}` +
      `.${String(d.getUTCMilliseconds()).padStart(3, "0")}`;
  }
  requestAnimationFrame(tickClock);
}

// ---- render coalescing ----
let renderQueued = false;
function scheduleRender(): void {
  if (renderQueued) return;
  renderQueued = true;
  requestAnimationFrame(() => {
    renderQueued = false;
    pruneStale();
    renderFix();
    drawSkyplot();
    drawSnr();
  });
}

// ---- connection ----
function setConn(up: boolean, text: string, src: string): void {
  const c = document.querySelector(".conn")!;
  c.classList.toggle("up", up);
  $("conn-text").textContent = text;
  $("conn-src").textContent = src;
}

function connect(): void {
  const ws = new WebSocket(`ws://${location.host}`);
  ws.onopen = () => setConn(true, "connected", "");
  ws.onclose = () => {
    setConn(false, "disconnected — retrying", "");
    setTimeout(connect, 1500);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    let m: Msg;
    try {
      m = JSON.parse(ev.data as string) as Msg;
    } catch {
      return;
    }
    switch (m.t) {
      case "nmea": handleNmea(m.s); break;
      case "pps": onPps(m); break;
      case "sync": onSync(m); break;
      case "status":
        setConn(m.connected, m.connected ? "streaming" : (m.note ?? "waiting for data"), m.source);
        break;
    }
  };
}

// ---- rate display ----
setInterval(() => {
  const now = performance.now();
  nmeaRate = (nmeaCount * 1000) / (now - nmeaWindowStart);
  $("rate-aux").textContent = `${nmeaRate.toFixed(1)} sentences/s`;
  nmeaCount = 0;
  nmeaWindowStart = now;
  scheduleRender();
}, 1000);

window.addEventListener("resize", scheduleRender);

// ---- boot ----
initMap();
renderFix();
scheduleRender();
requestAnimationFrame(tickClock);
connect();
