import { useEffect, useRef, useState } from "react";
import { Gnss } from "./nmea";
import type { GnssState, Msg } from "./types";

/** WebSocket に接続し、GNSS 状態を ~10fps のスナップショットで返す。 */
export function useGnss(): GnssState {
  const gnss = useRef(new Gnss());
  const [snap, setSnap] = useState<GnssState>(() => gnss.current.snapshot());

  useEffect(() => {
    let alive = true;
    let ws: WebSocket | null = null;

    const connect = () => {
      ws = new WebSocket(`ws://${location.host}`);
      ws.onopen = () => {
        gnss.current.conn = { ...gnss.current.conn, up: true, text: "connected" };
      };
      ws.onclose = () => {
        gnss.current.conn = { up: false, text: "disconnected — retrying", src: "" };
        if (alive) setTimeout(connect, 1500);
      };
      ws.onerror = () => ws?.close();
      ws.onmessage = (ev) => {
        let m: Msg;
        try {
          m = JSON.parse(ev.data as string) as Msg;
        } catch {
          return;
        }
        gnss.current.dispatch(m);
      };
    };
    connect();

    const id = window.setInterval(() => {
      gnss.current.prune(performance.now());
      setSnap(gnss.current.snapshot());
    }, 100);

    return () => {
      alive = false;
      clearInterval(id);
      ws?.close();
    };
  }, []);

  return snap;
}
