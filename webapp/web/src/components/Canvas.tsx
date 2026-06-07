import { useEffect, useRef } from "react";

/** dpr 対応の再利用キャンバス。親が再描画されるたび draw を呼ぶ。 */
export function Canvas({
  draw,
  className,
}: {
  draw: (ctx: CanvasRenderingContext2D, w: number, h: number) => void;
  className?: string;
}) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const c = ref.current;
    if (!c) return;
    const dpr = window.devicePixelRatio || 1;
    const rect = c.getBoundingClientRect();
    const w = Math.max(1, Math.round(rect.width));
    const h = Math.max(1, Math.round(rect.height));
    if (c.width !== w * dpr || c.height !== h * dpr) {
      c.width = w * dpr;
      c.height = h * dpr;
    }
    const ctx = c.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    draw(ctx, w, h);
  });
  return <canvas ref={ref} className={className} />;
}
