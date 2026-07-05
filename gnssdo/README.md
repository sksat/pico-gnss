# gnssdo

[![crates.io](https://img.shields.io/crates/v/gnssdo.svg)](https://crates.io/crates/gnssdo)
[![docs.rs](https://img.shields.io/docsrs/gnssdo)](https://docs.rs/gnssdo)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/sksat/pico-gnss/blob/main/LICENSE)

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
- `PhaseLockLoop` — a type-II output-phase servo (P/I/D + Smith predictor) for
  disciplining a *generated* 1PPS edge to the reference.
- `Gnssdo` — a turn-key easy tier bundling `PpsTracker` + `DisciplinedClock` under
  one good default discipline policy (feed-only-when-locked + holdover quarantine).

## Scope

This crate is just the **discipline**: it consumes a UTC epoch (`capture_ns ↔
unix_ns`, via `update_epoch` / `Gnssdo::on_utc`) and PPS intervals, and is agnostic to
*where* absolute time comes from. Decoding the time source (NMEA framing/parsing) and
pairing a PPS edge with its UTC second are a separate responsibility — on the RP2040
they live in the sibling [`rp-pps`](https://crates.io/crates/rp-pps) crate.

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

The crate has **no dependencies and no Cargo features** — it is pure integer logic.
NMEA decoding (with an optional `nmea`-crate backend) and the PPS↔UTC-second pairing
that produce the epoch you feed to `update_epoch` live in the sibling
[`rp-pps`](https://crates.io/crates/rp-pps) crate.
