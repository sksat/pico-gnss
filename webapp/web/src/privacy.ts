/**
 * 公開用 demo / privacy モードのヘルパ。スクショを共有する時に位置情報 (= 自宅) を一括で伏せる。
 * URL `?demo=1` でも起動でき、Header のトグルでも切替。
 */

/** NMEA 位置センテンスの緯度経度フィールド (カンマ split 後の index, [lat, lon])。 */
const LATLON_FIELDS: Record<string, [number, number]> = {
  GGA: [2, 4],
  RMC: [3, 5],
  GLL: [1, 3],
  GNS: [2, 4],
};

/** demo モードで NMEA の緯度経度の数値フィールドを伏せる (N/S/E/W や他フィールドは残し、構造は保つ)。 */
export function redactNmea(line: string): string {
  const parts = line.split(",");
  const fields = LATLON_FIELDS[parts[0]?.slice(3, 6) ?? ""];
  if (!fields) return line;
  for (const i of fields) if (parts[i]) parts[i] = "••••";
  return parts.join(",");
}

/** 緯度経度を伏せる表示。 */
export const MASK = "••••••";
