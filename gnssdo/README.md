# gnssdo

[![crates.io](https://img.shields.io/crates/v/gnssdo.svg)](https://crates.io/crates/gnssdo)
[![docs.rs](https://img.shields.io/docsrs/gnssdo)](https://docs.rs/gnssdo)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/sksat/pico-gnss-rs/blob/main/LICENSE)

`no_std`, HAL-agnostic **GNSS PPS disciplined clock & holdover core** (a GPSDO /
GNSSDO — GNSS-disciplined oscillator — building block). Integer-only,
zero-dependency core.

The 1 Hz PPS edge marks the UTC second boundary. Feed this crate integer-nanosecond
timestamps and it estimates your local crystal's frequency offset, keeps a
disciplined UTC epoch, and **extrapolates through holdover** while PPS is missing.
It is HAL-agnostic: you pass timestamps as plain integers, so it runs on any MCU
(RP2040, STM32, nRF, RISC-V, …) or on the host.

## What it provides

- `DisciplinedClock` — EMA-estimates the crystal frequency offset (ppb) from PPS
  intervals, with multi-stage gating + return-from-holdover quarantine; converts
  between a local timebase and disciplined UTC (and back), holdover-extrapolated.
- `PpsTracker` — classifies a PPS edge stream as lock / missed-pulse / glitch /
  non-monotonic. Tolerance and nominal rate are configurable.
- `PpsTimeSync` — pairs NMEA UTC with a PPS edge to establish a µs-precision UTC
  epoch. The PPS↔NMEA second association (±1 s, receiver-dependent) is a type
  (`PpsNmeaAssociation`) so the classic 1-second offset bug is hard to hit.
- Minimal NMEA helpers (`parse_rmc_time_date`, `parse_hhmmss`, `civil_to_unix`, …)
  for extracting time+date only — the core does **not** require a NMEA parser
  (`PpsTimeSync` takes already-parsed values), so you can bring your own.

## Two timebases

`DisciplinedClock` works with two device-clock timebases, both passed as integer ns:

- **capture timebase** — high-resolution PPS edge capture (RP2040 PIO; on other MCUs,
  a timer input-capture). Used for ns-precision error / `fire_at_utc`.
- **query timebase** — a continuously-readable clock (e.g. embassy `Instant`). Used
  for ticker / holdover queries.

## Example (sketch)

```rust
use gnssdo::{DisciplinedClock, PpsTracker, PpsEvent};

let mut tracker = PpsTracker::new();
let mut clock = DisciplinedClock::new();

// On each 1PPS rising edge (timestamps are integer ns from your own clock(s)):
if let PpsEvent::Locked { .. } = tracker.record(capture_ns / 1000) {
    clock.update_freq(interval_ns);            // estimate crystal offset (ppb), EMA
}
clock.update_epoch(capture_ns, query_ns, utc_unix_ns); // anchor UTC epoch to the edge

// Any time afterwards — frequency-corrected, holdover-extrapolated UTC:
let utc_ns = clock.now_from_query_ns(query_ns);
```

Tuning (`DisciplinedClockConfig`, `PpsTrackerConfig`) is exposed; defaults are the
measured settling values from a GYSFFMANC (MT3333) receiver. Invalid settings are
made unrepresentable via `NonZero*` types.

## Cargo features

- **`external-nmea`** (off by default): delegate `parse_rmc_time_date`'s RMC parsing
  to the [`nmea`](https://docs.rs/nmea) crate instead of the built-in parser. The
  default in-house parser is zero-dependency and tiny; enable this only if you want
  NMEA checksum validation or already depend on `nmea`. On an RP2040 the `nmea`-backed
  path is ~17× slower per sentence and adds ~52 KB of `.text` — negligible at 1 Hz,
  but the default keeps the core dependency-free.

  Backend differences: the `nmea` crate validates checksums, interprets the year with
  a century pivot (`yy=94`→1994), and rejects leap-second `ss=60`; the built-in parser
  skips checksums, assumes 20xx, and accepts `ss=60`.
