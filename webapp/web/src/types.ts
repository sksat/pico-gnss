export type Msg =
  | { t: "nmea"; s: string }
  | { t: "pps"; count: number; interval_us: number; interval_ns: number; state: string; missed: number }
  | { t: "sync"; pps_local_us: number; unix_s: number; drift_us: number; err_ns: number; holdover_ms: number }
  | { t: "time"; unix_ns: number; ppb: number; holdover_ms: number; locked: boolean }
  | { t: "ppsout"; unix_s: number; late_us: number; holdover_ms: number }
  | { t: "ppsgen"; count: number; interval_ns: number; dev_ns: number; phase_ns: number }
  | { t: "fw"; s: string }
  | { t: "status"; source: string; connected: boolean; note?: string };

export interface Sat {
  sys: string;
  prn: number;
  elev: number | null;
  azim: number | null;
  snr: number | null;
  seen: number;
}
export interface Fix {
  lat: number | null;
  lon: number | null;
  alt: number | null;
  satsUsed: number | null;
  hdop: number | null;
  pdop: number | null;
  vdop: number | null;
  quality: number;
  mode: number; // GSA fix type: 1=none 2=2D 3=3D
  speedKn: number | null;
  courseDeg: number | null;
  status: string;
}
/** GST: 受信機が報告する測位の標準偏差 (m)。 */
export interface Gst {
  rms: number | null;
  sLat: number | null;
  sLon: number | null;
  sAlt: number | null;
}
export interface Pps {
  count: number;
  interval_us: number;
  interval_ns: number;
  dev: number; // interval_ns - 1e9 (ns)
  state: string;
  missed: number;
}
export interface Sync {
  unix_s: number;
  pps_local_us: number;
  drift_us: number;
  err_ns: number; // 補正後の 1 秒先読み残差 (時刻補正の精度)
  holdover_ms: number; // この err が何秒 holdover の誤差か (通常 ~1000)
  wall: number;
}
/** holdover 経過 (s) と その時の補正後誤差 (ns) の 1 点。 */
export interface HoldoverPt {
  h: number; // holdover 経過 (秒)
  e: number; // 補正後 clock err (ns)
}
/** GPSDO の規律クロック状態 (firmware の TIME 行)。 */
export interface Gpsdo {
  unixMs: number; // 規律 UTC (ms, 受信時点)
  ppb: number; // 推定周波数オフセット (ns/s)
  holdoverMs: number; // 最後の PPS 規律からの経過
  locked: boolean;
  wall: number; // 受信時の Date.now()
}
/** 規律 PPS 出力 (GP3) の状態。 */
export interface PpsOut {
  unix_s: number;
  late_us: number; // スケジュール時刻からの遅れ (executor ジッタ)
  holdover_ms: number;
  wall: number;
}
/** PIO 規律 PPS 生成出力 (GP3→GP4 ループバック計測)。 */
export interface PpsGen {
  count: number;
  dev_ns: number; // 出力周期 - 1e9 (SM2 計測, ローカル counter なので +ppm 込み)
  jitter_ns: number; // 直近の周期ばらつき (peak-peak)
  phase_ns: number; // UTC 秒境界からの位相ズレ (ソフト同期, ~ms 精度)
}
export interface RawLine {
  ts: string;
  sentence: string;
  kind: string;
}
export interface PosSample {
  t: number;
  lat: number;
  lon: number;
  alt: number | null;
}
export interface TrackSample {
  t: number;
  alt: number | null;
  speedKmh: number | null;
  satsUsed: number | null;
  inView: number | null;
}
export interface Conn {
  up: boolean;
  text: string;
  src: string;
}
export interface GnssState {
  fix: Fix;
  sats: Sat[];
  usedPrn: Set<number>;
  pps: Pps | null;
  ppsDev: number[];
  sync: Sync | null;
  errHist: number[]; // clock err (ns) の履歴
  holdoverPts: HoldoverPt[]; // (holdover 秒, err ns) 散布
  gpsdo: Gpsdo | null;
  ppsOut: PpsOut | null;
  ppsGen: PpsGen | null;
  gst: Gst | null;
  fw: string | null;
  posHist: PosSample[];
  track: TrackSample[];
  raw: RawLine[];
  conn: Conn;
}
