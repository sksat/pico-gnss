import { useEffect, useRef } from "react";
import { CircleMarker, MapContainer, Polyline, TileLayer, useMap } from "react-leaflet";
import type { GnssState } from "../types";

type LatLng = [number, number];

/**
 * 初回 fix で 1 回だけ中心合わせ。以後はズームを一切自動変更せず、ユーザが地図を
 * ドラッグするまで pan で追従するだけ (ズームは常に保持・ユーザ操作を尊重)。
 */
function Follow({ pos, demo }: { pos: LatLng | null; demo: boolean }) {
  const map = useMap();
  const inited = useRef(false);
  const userHeld = useRef(false);
  useEffect(() => {
    setTimeout(() => map.invalidateSize(), 0);
    const onDrag = () => { userHeld.current = true; }; // 手動ドラッグしたら追従停止
    map.on("dragstart", onDrag);
    return () => { map.off("dragstart", onDrag); };
  }, [map]);
  useEffect(() => {
    if (demo) { map.setView([37.5, 137.0], 5); return; } // privacy: 位置を伏せ広域 (日本全体) へ
    if (!pos) return;
    if (!inited.current) {
      map.setView(pos, 17);
      inited.current = true;
    } else if (!userHeld.current) {
      map.panTo(pos, { animate: false }); // ズーム保持のまま中心追従
    }
  }, [pos?.[0], pos?.[1], demo]);
  return null;
}

export function MapPanel({ s, demo }: { s: GnssState; demo: boolean }) {
  const pos: LatLng | null = s.fix.lat != null && s.fix.lon != null ? [s.fix.lat, s.fix.lon] : null;
  const trail: LatLng[] = s.posHist.map((p) => [p.lat, p.lon]);
  return (
    <section className="panel a-map">
      <h2>Position <span className="hdr-aux">{demo ? "🔒 hidden" : pos ? `${pos[0].toFixed(6)}, ${pos[1].toFixed(6)}` : "no fix"}</span></h2>
      <MapContainer id="map" center={demo ? [37.5, 137.0] : [35.681236, 139.767125]} zoom={demo ? 5 : 16} preferCanvas>
        <TileLayer
          url="https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png"
          attribution="© OpenStreetMap · © CARTO"
          maxZoom={20}
        />
        {/* demo モードでは軌跡・マーカーを描かず (位置を伏せる)、Follow が広域へ。 */}
        {!demo && trail.length > 1 && <Polyline positions={trail} pathOptions={{ color: "#2563eb", weight: 2, opacity: 0.85 }} />}
        {!demo && pos && <CircleMarker center={pos} radius={7} pathOptions={{ color: "#0369a1", weight: 2, fillColor: "#38bdf8", fillOpacity: 0.7 }} />}
        <Follow pos={pos} demo={demo} />
      </MapContainer>
    </section>
  );
}
