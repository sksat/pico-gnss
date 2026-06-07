export type Msg =
  | { t: "nmea"; s: string }
  | { t: "pps"; count: number; interval_us: number; interval_ns: number; state: string; missed: number }
  | { t: "sync"; pps_local_us: number; unix_s: number; drift_us: number }
  | { t: "time"; unix_ns: number; ppb: number; holdover_ms: number; locked: boolean }
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
  wall: number;
}
/** GPSDO の規律クロック状態 (firmware の TIME 行)。 */
export interface Gpsdo {
  unixMs: number; // 規律 UTC (ms, 受信時点)
  ppb: number; // 推定周波数オフセット (ns/s)
  holdoverMs: number; // 最後の PPS 規律からの経過
  locked: boolean;
  wall: number; // 受信時の Date.now()
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
  gpsdo: Gpsdo | null;
  posHist: PosSample[];
  track: TrackSample[];
  raw: RawLine[];
  conn: Conn;
}
