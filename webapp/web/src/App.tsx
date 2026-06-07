import { useMemo } from "react";
import { useGnss } from "./useGnss";
import { computeAccuracy, computeTiming } from "./stats";
import { AccuracyPanel, SkyPlot, SnrChart, TimeSeries, TimingPanel } from "./components/charts";
import { ConsolePanel, FixPanel, Header, PpsPanel, SatTable, SyncPanel } from "./components/panels";
import { MapPanel } from "./components/MapPanel";

export function App() {
  const s = useGnss();
  const acc = useMemo(() => computeAccuracy(s.posHist), [s.posHist]);
  const timing = useMemo(() => computeTiming(s.ppsDev), [s.ppsDev]);

  return (
    <>
      <Header s={s} acc={acc} timing={timing} />
      <main>
        <section className="grid">
          <FixPanel s={s} />
          <PpsPanel s={s} />
          <SyncPanel s={s} />
          <MapPanel s={s} />
          <SkyPlot s={s} />
          <SnrChart s={s} />
        </section>

        <section className="grid-detail">
          <AccuracyPanel s={s} acc={acc} />
          <TimingPanel s={s} timing={timing} />
          <TimeSeries s={s} />
        </section>

        <SatTable s={s} />
        <ConsolePanel s={s} />
      </main>
    </>
  );
}
