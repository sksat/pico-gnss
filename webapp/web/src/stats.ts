import type { PosSample } from "./types";

export function mean(a: number[]): number {
  return a.length ? a.reduce((s, x) => s + x, 0) / a.length : 0;
}
export function std(a: number[]): number {
  if (a.length < 2) return 0;
  const m = mean(a);
  return Math.sqrt(a.reduce((s, x) => s + (x - m) ** 2, 0) / a.length);
}

export interface Accuracy {
  n: number;
  drms: number; // 1σ horizontal (~63%)
  twodrms: number; // 95%
  cep: number; // 50%
  sE: number;
  sN: number;
  sAlt: number;
  pts: { e: number; n: number }[]; // 平均からの East/North 偏差 (m)
}

const M_PER_DEG = 111_320;

/** 直近の測位点群から水平精度 (経験的なばらつき) を求める。 */
export function computeAccuracy(pos: PosSample[]): Accuracy {
  const n = pos.length;
  if (n < 2) return { n, drms: 0, twodrms: 0, cep: 0, sE: 0, sN: 0, sAlt: 0, pts: [] };
  const mLat = mean(pos.map((p) => p.lat));
  const mLon = mean(pos.map((p) => p.lon));
  const cosLat = Math.cos((mLat * Math.PI) / 180);
  const es = pos.map((p) => (p.lon - mLon) * M_PER_DEG * cosLat);
  const ns = pos.map((p) => (p.lat - mLat) * M_PER_DEG);
  const sE = std(es);
  const sN = std(ns);
  const drms = Math.sqrt(sE * sE + sN * sN);
  const alts = pos.map((p) => p.alt).filter((a): a is number => a != null);
  return {
    n,
    drms,
    twodrms: 2 * drms,
    cep: 0.59 * (sE + sN),
    sE,
    sN,
    sAlt: std(alts),
    pts: pos.map((_, i) => ({ e: es[i]!, n: ns[i]! })),
  };
}

export interface Timing {
  n: number;
  meanInterval: number; // µs
  sigma: number; // jitter 1σ (µs)
  pp: number; // peak-peak (µs)
  ppm: number; // 局部発振器オフセット
}

/** PPS 間隔偏差 (interval-1e6) の列から時刻精度指標を求める。 */
export function computeTiming(dev: number[]): Timing {
  const n = dev.length;
  if (n < 2) return { n, meanInterval: 0, sigma: 0, pp: 0, ppm: 0 };
  const m = mean(dev);
  return {
    n,
    meanInterval: 1_000_000 + m,
    sigma: std(dev),
    pp: Math.max(...dev) - Math.min(...dev),
    ppm: m, // µs/s = ppm
  };
}
