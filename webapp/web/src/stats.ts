import type { GnssState, PosSample } from "./types";

export function mean(a: number[]): number {
  return a.length ? a.reduce((s, x) => s + x, 0) / a.length : 0;
}
export function std(a: number[]): number {
  if (a.length < 2) return 0;
  const m = mean(a);
  return Math.sqrt(a.reduce((s, x) => s + (x - m) ** 2, 0) / a.length);
}

export interface Accuracy {
  n: number; // 計算に使った窓内サンプル数
  total: number; // 蓄積総数
  cep: number; // 経験的 50% 半径 (m)
  r95: number; // 経験的 95% 半径 (m)
  drms: number; // √(σE²+σN²) (1σ ~63%)
  twodrms: number; // 2×DRMS (~95%, 正規分布近似)
  sE: number;
  sN: number;
  sAlt: number;
  pts: { e: number; n: number }[]; // 窓内の East/North 偏差 (m)
}

const M_PER_DEG = 111_320;
// 直近ウィンドウだけで評価する (cold start 直後の収束ジャンプを除き、現在の条件を反映)。
const WINDOW = 120; // ~2 min @ 1Hz

export function computeAccuracy(all: PosSample[]): Accuracy {
  const pos = all.slice(-WINDOW);
  const n = pos.length;
  const empty: Accuracy = { n, total: all.length, cep: 0, r95: 0, drms: 0, twodrms: 0, sE: 0, sN: 0, sAlt: 0, pts: [] };
  if (n < 3) return empty;
  const mLat = mean(pos.map((p) => p.lat));
  const mLon = mean(pos.map((p) => p.lon));
  const cosLat = Math.cos((mLat * Math.PI) / 180);
  const es = pos.map((p) => (p.lon - mLon) * M_PER_DEG * cosLat);
  const ns = pos.map((p) => (p.lat - mLat) * M_PER_DEG);
  const sE = std(es);
  const sN = std(ns);
  const drms = Math.sqrt(sE * sE + sN * sN);
  const radii = es.map((e, i) => Math.hypot(e, ns[i]!)).sort((a, b) => a - b);
  const pct = (p: number) => radii[Math.min(radii.length - 1, Math.floor(p * radii.length))]!;
  const alts = pos.map((p) => p.alt).filter((a): a is number => a != null);
  return {
    n, total: all.length,
    cep: pct(0.5), r95: pct(0.95),
    drms, twodrms: 2 * drms,
    sE, sN, sAlt: std(alts),
    pts: pos.map((_, i) => ({ e: es[i]!, n: ns[i]! })),
  };
}

export interface Timing {
  n: number;
  meanIntervalNs: number; // ns
  sigma: number; // jitter 1σ (ns) — PIO ハードキャプチャの分解能 ~16ns
  pp: number; // peak-peak (ns)
  ppm: number; // 局部発振器オフセット (ns/s = 1000ppm → ppm = mean/1000)
}

/** PPS 間隔偏差 (interval_ns - 1e9) の列から時刻精度指標を求める。単位は ns。 */
export function computeTiming(devNs: number[]): Timing {
  const n = devNs.length;
  if (n < 2) return { n, meanIntervalNs: 0, sigma: 0, pp: 0, ppm: 0 };
  const m = mean(devNs);
  return {
    n,
    meanIntervalNs: 1_000_000_000 + m,
    sigma: std(devNs),
    pp: Math.max(...devNs) - Math.min(...devNs),
    ppm: m / 1000,
  };
}

// 受信条件の良し悪し = タイミング精度 (PPS σ) の上限。各要因を 0/1/2 で採点し色分けする。
const REC_COLOR = ["#f87272" /*poor*/, "#fbbd23" /*fair*/, "#36d399" /*good*/] as const;
export interface RecFactor {
  k: string;
  text: string;
  color: string;
}
export interface Reception {
  verdict: string; // GOOD / FAIR / POOR / NO FIX
  cls: string; // good / fair / poor (CSS)
  factors: RecFactor[];
}

/**
 * fix / 衛星数 / 幾何 (HDOP) / 信号 (C/N₀) から受信条件を一目で評価する。
 * 結果として律速される PPS jitter (σ) も末尾に並べる (これは結果であって採点には入れない)。
 */
export function assessReception(s: GnssState, timing: Timing): Reception {
  const f = s.fix;
  const col = (lvl: number) => REC_COLOR[lvl] ?? "#7a8a9a";

  let fixLvl: number;
  let fixText: string;
  if (f.mode === 3) {
    fixLvl = 2;
    fixText = f.quality >= 2 ? "3D + SBAS" : "3D fix";
  } else if (f.mode === 2) {
    fixLvl = 1;
    fixText = "2D fix";
  } else {
    fixLvl = 0;
    fixText = "no fix";
  }

  const used = f.satsUsed ?? 0;
  const satLvl = used >= 8 ? 2 : used >= 5 ? 1 : 0;

  const h = f.hdop;
  const hdopLvl = h == null ? 0 : h < 2 ? 2 : h < 5 ? 1 : 0;

  const snrs = s.sats.filter((x) => s.usedPrn.has(x.prn) && x.snr != null && x.snr > 0).map((x) => x.snr!);
  const cn0 = snrs.length ? snrs.reduce((a, b) => a + b, 0) / snrs.length : 0;
  const cn0Lvl = snrs.length === 0 ? 0 : cn0 >= 38 ? 2 : cn0 >= 30 ? 1 : 0;

  // 結果指標 (PPS interval σ)。採点には入れず、上限の現れとして表示。
  const sig = timing.sigma;
  const sigKnown = timing.n >= 5;
  const sigLvl = sig < 100 ? 2 : sig < 1000 ? 1 : 0;
  const sigText = !sigKnown ? "—" : Math.abs(sig) < 1000 ? `±${sig.toFixed(0)} ns` : `±${(sig / 1000).toFixed(2)} µs`;

  const noFix = f.mode < 2;
  const sum = fixLvl + satLvl + hdopLvl + cn0Lvl; // 0..8
  const verdict = noFix ? "NO FIX" : sum >= 7 ? "GOOD" : sum >= 4 ? "OK" : "POOR";
  const cls = noFix || sum < 4 ? "poor" : sum >= 7 ? "good" : "fair";

  return {
    verdict,
    cls,
    factors: [
      { k: "Fix", text: fixText, color: col(fixLvl) },
      { k: "Sats used", text: `${used} used / ${s.sats.length} in view`, color: col(satLvl) },
      { k: "HDOP", text: h != null ? h.toFixed(1) : "—", color: col(hdopLvl) },
      { k: "C/N₀ (used)", text: snrs.length ? `${cn0.toFixed(0)} dB-Hz avg` : "—", color: col(cn0Lvl) },
      { k: "→ PPS jitter", text: sigText, color: sigKnown ? col(sigLvl) : "#7a8a9a" },
    ],
  };
}
