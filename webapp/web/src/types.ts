export type Msg =
  | { t: "nmea"; s: string }
  | { t: "pps"; count: number; interval_us: number; state: string; missed: number }
  | { t: "sync"; pps_local_us: number; unix_s: number; drift_us: number }
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
  dev: number;
  state: string;
  missed: number;
}
export interface Sync {
  unix_s: number;
  pps_local_us: number;
  drift_us: number;
  wall: number;
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
  posHist: PosSample[];
  track: TrackSample[];
  raw: RawLine[];
  conn: Conn;
}
