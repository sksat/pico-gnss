import { useEffect } from "react";
import { CircleMarker, MapContainer, Polyline, TileLayer, useMap } from "react-leaflet";
import type { GnssState } from "../types";

type LatLng = [number, number];

/** 現在地が更新されたら追従。初回は invalidateSize でサイズを確定させる。 */
function Follow({ pos }: { pos: LatLng | null }) {
  const map = useMap();
  useEffect(() => {
    setTimeout(() => map.invalidateSize(), 0);
  }, [map]);
  useEffect(() => {
    if (!pos) return;
    if (map.getZoom() < 14) map.setView(pos, 17);
    else map.panTo(pos, { animate: true, duration: 0.4 });
  }, [pos?.[0], pos?.[1]]);
  return null;
}

export function MapPanel({ s }: { s: GnssState }) {
  const pos: LatLng | null = s.fix.lat != null && s.fix.lon != null ? [s.fix.lat, s.fix.lon] : null;
  const trail: LatLng[] = s.posHist.map((p) => [p.lat, p.lon]);
  return (
    <section className="panel a-map">
      <h2>Position <span className="hdr-aux">{pos ? `${pos[0].toFixed(6)}, ${pos[1].toFixed(6)}` : "no fix"}</span></h2>
      <MapContainer id="map" center={[35.681236, 139.767125]} zoom={16} preferCanvas>
        <TileLayer
          url="https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png"
          attribution="© OpenStreetMap · © CARTO"
          maxZoom={20}
        />
        {trail.length > 1 && <Polyline positions={trail} pathOptions={{ color: "#2563eb", weight: 2, opacity: 0.85 }} />}
        {pos && <CircleMarker center={pos} radius={7} pathOptions={{ color: "#0369a1", weight: 2, fillColor: "#38bdf8", fillOpacity: 0.7 }} />}
        <Follow pos={pos} />
      </MapContainer>
    </section>
  );
}
