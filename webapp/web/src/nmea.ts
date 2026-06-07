import type { Fix, GnssState, Msg, Sat } from "./types";

export const SYS_INFO: Record<string, { name: string; color: string }> = {
  GPS: { name: "GPS", color: "#38bdf8" },
  GLONASS: { name: "GLONASS", color: "#f472b6" },
  Galileo: { name: "Galileo", color: "#a78bfa" },
  BeiDou: { name: "BeiDou", color: "#fbbf24" },
  QZSS: { name: "QZSS", color: "#34d399" },
  SBAS: { name: "SBAS", color: "#fb923c" },
  Other: { name: "Other", color: "#94a3b8" },
};
export function sysColor(sys: string): string {
  return SYS_INFO[sys]?.color ?? "#94a3b8";
}
export function snrColor(snr: number | null): string {
  if (snr == null || snr <= 0) return "#3a4654";
  if (snr < 20) return "#f87272";
  if (snr < 30) return "#fbbd23";
  if (snr < 38) return "#a3e635";
  return "#36d399";
}

/**
 * コンステレーション判定。talker だけでなく PRN レンジも見る。
 * GYSFFMANC は QZSS (みちびき) を $GPGSV talker で PRN 193+ として出すため、
 * talker=GP でも PRN で QZSS/SBAS を切り分ける。
 */
export function classify(talker: string, prn: number): string {
  switch (talker) {
    case "GL": return "GLONASS";
    case "GA": return "Galileo";
    case "GB":
    case "BD": return "BeiDou";
    case "GQ": return "QZSS";
  }
  // GP / GN
  if (prn >= 1 && prn <= 32) return "GPS";
  if (prn >= 33 && prn <= 64) return "SBAS";
  if (prn >= 120 && prn <= 158) return "SBAS";
  if ((prn >= 193 && prn <= 202) || (prn >= 183 && prn <= 192)) return "QZSS";
  if (prn >= 65 && prn <= 96) return "GLONASS";
  if (prn >= 301 && prn <= 336) return "Galileo";
  return "Other";
}

function num(s: string | undefined): number | null {
  if (s == null || s === "") return null;
  const v = Number(s);
  return Number.isFinite(v) ? v : null;
}
function coord(val: string, hemi: string): number | null {
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

const SAT_STALE_MS = 8000;
const POS_CAP = 600;

function emptyFix(): Fix {
  return {
    lat: null, lon: null, alt: null, satsUsed: null, hdop: null, pdop: null,
    vdop: null, quality: 0, mode: 1, speedKn: null, courseDeg: null, status: "V",
  };
}

/** WebSocket メッセージを取り込み、GNSS 状態を蓄積する (mutable, 高頻度更新用)。 */
export class Gnss {
  fix: Fix = emptyFix();
  sats = new Map<string, Sat>();
  usedPrn = new Map<number, number>();
  pps: GnssState["pps"] = null;
  ppsDev: number[] = [];
  sync: GnssState["sync"] = null;
  posHist: GnssState["posHist"] = [];
  track: GnssState["track"] = [];
  raw: GnssState["raw"] = [];
  conn: GnssState["conn"] = { up: false, text: "connecting…", src: "" };
  private inView: number | null = null;

  dispatch(m: Msg): void {
    switch (m.t) {
      case "nmea": this.onNmea(m.s); break;
      case "pps":
        this.pps = { count: m.count, interval_us: m.interval_us, dev: m.interval_us - 1_000_000, state: m.state, missed: m.missed };
        if (m.interval_us > 0) {
          this.ppsDev.push(m.interval_us - 1_000_000);
          if (this.ppsDev.length > 600) this.ppsDev.shift();
        }
        break;
      case "sync":
        this.sync = { unix_s: m.unix_s, pps_local_us: m.pps_local_us, drift_us: m.drift_us, wall: Date.now() };
        break;
      case "status":
        this.conn = { up: m.connected, text: m.connected ? "streaming" : (m.note ?? "waiting for data"), src: m.source };
        break;
    }
  }

  private onNmea(sentence: string): void {
    const star = sentence.indexOf("*");
    const body = star >= 0 ? sentence.slice(0, star) : sentence;
    const p = body.split(",");
    const type0 = p[0] ?? "";
    const talker = type0.slice(1, 3);
    const kind = type0.slice(3);
    const now = performance.now();

    switch (kind) {
      case "GGA": {
        const lat = coord(p[2] ?? "", p[3] ?? "");
        const lon = coord(p[4] ?? "", p[5] ?? "");
        this.fix.quality = num(p[6]) ?? 0;
        this.fix.satsUsed = num(p[7]);
        this.fix.hdop = num(p[8]);
        this.fix.alt = num(p[9]);
        if (this.fix.quality > 0 && lat != null && lon != null) {
          this.fix.lat = lat;
          this.fix.lon = lon;
          this.posHist.push({ t: Date.now(), lat, lon, alt: this.fix.alt });
          if (this.posHist.length > POS_CAP) this.posHist.shift();
          this.track.push({
            t: Date.now(), alt: this.fix.alt,
            speedKmh: this.fix.speedKn != null ? this.fix.speedKn * 1.852 : null,
            satsUsed: this.fix.satsUsed, inView: this.inView,
          });
          if (this.track.length > POS_CAP) this.track.shift();
        } else if (this.fix.quality === 0) {
          this.fix.lat = null;
          this.fix.lon = null;
        }
        break;
      }
      case "RMC": {
        this.fix.status = p[2] ?? "V";
        this.fix.speedKn = num(p[7]);
        this.fix.courseDeg = num(p[8]);
        break;
      }
      case "VTG": {
        this.fix.courseDeg = num(p[1]) ?? this.fix.courseDeg;
        this.fix.speedKn = num(p[5]) ?? this.fix.speedKn;
        break;
      }
      case "GSA": {
        this.fix.mode = num(p[2]) ?? this.fix.mode;
        this.fix.pdop = num(p[15]);
        this.fix.hdop = num(p[16]) ?? this.fix.hdop;
        this.fix.vdop = num(p[17]);
        for (let i = 3; i <= 14; i++) {
          const prn = num(p[i]);
          if (prn != null) this.usedPrn.set(prn, now);
        }
        break;
      }
      case "GSV": {
        const iv = num(p[3]);
        if (iv != null) this.inView = iv;
        for (let i = 4; i + 3 < p.length; i += 4) {
          const prn = num(p[i]);
          if (prn == null) continue;
          const sys = classify(talker, prn);
          this.sats.set(sys + prn, {
            sys, prn,
            elev: num(p[i + 1]),
            azim: num(p[i + 2]),
            snr: num(p[i + 3]),
            seen: now,
          });
        }
        break;
      }
    }

    const d = new Date();
    const ts = `${String(d.getHours()).padStart(2, "0")}:${String(d.getMinutes()).padStart(2, "0")}:${String(d.getSeconds()).padStart(2, "0")}`;
    this.raw.push({ ts, sentence, kind });
    if (this.raw.length > 400) this.raw.shift();
  }

  prune(now: number): void {
    for (const [k, s] of this.sats) if (now - s.seen > SAT_STALE_MS) this.sats.delete(k);
    for (const [p, t] of this.usedPrn) if (now - t > SAT_STALE_MS) this.usedPrn.delete(p);
  }

  snapshot(): GnssState {
    return {
      fix: { ...this.fix },
      sats: [...this.sats.values()],
      usedPrn: new Set(this.usedPrn.keys()),
      pps: this.pps,
      ppsDev: this.ppsDev.slice(-400),
      sync: this.sync,
      posHist: this.posHist.slice(),
      track: this.track.slice(),
      raw: this.raw.slice(-250),
      conn: { ...this.conn },
    };
  }
}
